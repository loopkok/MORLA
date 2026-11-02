"""One composed MuJoCo scene shared by all simulation actuators.

Physics stepping and state sampling are independent from runtime command ingress.
This reuses the installed robot_description builder and preserves its explicit
joint map and honest collision/model validation level. No physical SDK is loaded.
"""
import copy
import importlib.util
from pathlib import Path
import threading
import time


def _builder():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('robot_description')) / 'scripts/composable_scene.py'
    except (ImportError, LookupError):
        # Fixed source-checkout location, never a path from AssemblySpec.
        path = Path(__file__).resolve().parents[3] / 'robot_description/scripts/composable_scene.py'
    if not path.is_file():
        raise RuntimeError('COMPOSED_SIM_BUILDER_NOT_INSTALLED')
    spec = importlib.util.spec_from_file_location('morla_trusted_composable_scene', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build_scene


class MujocoDeviceSession:
    def __init__(self, runtime, builder=None):
        self.runtime = runtime
        self.builder = builder or _builder()
        self.configured = False
        self.command_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.pending = {}
        self.active = set()
        self.cache = {}
        self.camera_cache = {}
        self.view_lock = threading.Lock()
        self.view_actions = {}
        self.render_error = None
        self.error = None
        self.closed = False

    def configure(self):
        if self.configured: return
        import mujoco
        import numpy as np
        actuator_rows = [row for row in self.runtime.instances.values() if row.get('kind') == 'actuator']
        if any(row.get('backend') != 'sim' for row in actuator_rows):
            raise ValueError('COMPOSED_SIM_REQUIRES_ALL_ACTUATORS_IN_SAME_SCENE')
        self.artifact = self.builder(self.runtime.resolved)
        self.mujoco, self.np = mujoco, np
        self.model = mujoco.MjModel.from_xml_string(self.artifact['xml'])
        self.data = mujoco.MjData(self.model)
        self.indices = {}
        for row in actuator_rows:
            key = row['instance_id']
            names = [joint['name'] for joint in row['joints']]
            mapping = self.artifact['joint_map'][key]
            joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, mapping[name]) for name in names]
            if any(index < 0 for index in joint_ids):
                raise ValueError('SIM_JOINT_MAPPING_MISSING')
            controls = []
            for joint_id in joint_ids:
                matches = [index for index in range(self.model.nu) if self.model.actuator_trnid[index, 0] == joint_id]
                if len(matches) != 1:
                    raise ValueError('SIM_REQUIRES_UNAMBIGUOUS_JOINT_ACTUATOR')
                controls.append(matches[0])
            qpos = [int(self.model.jnt_qposadr[index]) for index in joint_ids]
            qvel = [int(self.model.jnt_dofadr[index]) for index in joint_ids]
            self.indices[key] = {'qpos': qpos, 'qvel': qvel, 'ctrl': controls}
            initial = row.get('parameters', {}).get('initial_positions')
            if initial is not None:
                if len(initial) != len(names) or not np.isfinite(initial).all():
                    raise ValueError('SIM_INVALID_INITIAL_POSITIONS')
                for value, joint in zip(initial, row['joints']):
                    if not joint['lower'] <= value <= joint['upper']:
                        raise ValueError('SIM_INITIAL_POSITION_OUTSIDE_LIMITS')
                self.data.qpos[qpos] = initial
            self.data.ctrl[controls] = self.data.qpos[qpos]
        self.cameras = [row for row in self.runtime.instances.values()
                        if row.get('backend') == 'sim' and row.get('kind') == 'sensor'
                        and row['plugin_id'].startswith('morla.camera')]
        self.camera_ids = {row['instance_id']: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, row['instance_id'] + '__rgb') for row in self.cameras}
        if any(value < 0 for value in self.camera_ids.values()):
            raise ValueError('SIM_CAMERA_MAPPING_MISSING')
        mujoco.mj_forward(self.model, self.data)
        self.configured = True
        self._snapshot()
        self.thread = threading.Thread(target=self._step, name='morla-mujoco-scene', daemon=True)
        self.thread.start()
        if self.cameras:
            self.render_thread = threading.Thread(target=self._render, name='morla-mujoco-render', daemon=True)
            self.render_thread.start()

    def _snapshot(self):
        now = self.runtime.clock()
        values = {}
        for key, indices in self.indices.items():
            position = self.data.qpos[indices['qpos']].tolist()
            velocity = self.data.qvel[indices['qvel']].tolist()
            if not self.np.isfinite(position).all() or not self.np.isfinite(velocity).all():
                raise ValueError('SIM_NONFINITE_STATE')
            sample = {'ready': True, 'sample_ns': now, 'source': 'simulation', 'measured': False,
                      'synthetic': True, 'values': position, 'simulation_time_s': float(self.data.time),
                      'simulation_level': self.artifact['simulation_level'],
                      'scene_id': id(self), 'ports': {}}
            for name, vector, unit in (('position', position, 'rad'), ('velocity', velocity, 'rad/s')):
                sample['ports'][name] = {'values': vector, 'sample_ns': now, 'source': 'simulation',
                                         'valid': True, 'measured': False, 'synthetic': True, 'unit': unit}
            values[key] = sample
        with self.state_lock:
            self.cache = values
            self.qpos_snapshot = self.data.qpos.copy()
            self.time_snapshot = float(self.data.time)

    def _step(self):
        try:
            period = float(self.model.opt.timestep)
            while not self.stop_event.is_set():
                began = time.monotonic()
                with self.command_lock:
                    pending, self.pending = self.pending, {}
                for key, target in pending.items():
                    self.data.ctrl[self.indices[key]['ctrl']] = target
                self.mujoco.mj_step(self.model, self.data)
                self._snapshot()
                self.stop_event.wait(max(0, period - (time.monotonic() - began)))
        except Exception as exc:
            self.error = str(exc)
            self.stop_event.set()

    def _render(self):
        # Offscreen rendering/encoding has its own context and MjData. It never
        # holds the stepping or command lock while graphics/codec work runs.
        renderers = {}
        views = {}
        try:
            import io
            from PIL import Image
            local_data = self.mujoco.MjData(self.model)
            last = {}
            def free_camera(key):
                # The preview starts as an orbit camera aimed exactly like the
                # configured camera: MuJoCo free cameras pivot on lookat, so
                # plant lookat on the existing view ray (the world origin for
                # the shipped scenes) with distance = the pivot distance.
                position = local_data.cam_xpos[self.camera_ids[key]].copy()
                forward = -local_data.cam_xmat[self.camera_ids[key]].reshape(3, 3)[:, 2]
                distance = max(.5, float(-position @ forward))
                camera = self.mujoco.MjvCamera()
                camera.type = self.mujoco.mjtCamera.mjCAMERA_FREE
                camera.azimuth = float(self.np.degrees(self.np.arctan2(forward[1], forward[0])))
                camera.elevation = float(self.np.degrees(self.np.arcsin(self.np.clip(forward[2], -1, 1))))
                camera.distance = distance
                camera.lookat[:] = position + forward * distance
                views[key] = camera
                return camera
            def move_camera(key, camera, size):
                with self.view_lock:
                    actions = self.view_actions.pop(key, [])
                mouse = self.mujoco.mjtMouse
                for action, dx, dy in actions:
                    if action == 'reset':
                        free_camera(key)
                        return views[key]
                    if action == 'orbit':
                        self.mujoco.mjv_moveCamera(self.model, mouse.mjMOUSE_ROTATE_H, dx / size[0], 0., camera)
                        self.mujoco.mjv_moveCamera(self.model, mouse.mjMOUSE_ROTATE_V, 0., dy / size[1], camera)
                    elif action == 'pan':
                        self.mujoco.mjv_moveCamera(self.model, mouse.mjMOUSE_MOVE_H, dx / size[0], 0., camera)
                        self.mujoco.mjv_moveCamera(self.model, mouse.mjMOUSE_MOVE_V, 0., dy / size[1], camera)
                    elif action == 'zoom':
                        # Wheel down (positive dy) zooms out; a quarter weight
                        # keeps one notch near 10% instead of jumping to a new
                        # distance.
                        self.mujoco.mjv_moveCamera(self.model, mouse.mjMOUSE_ZOOM, 0., -dy / size[1] * .25, camera)
                return camera
            while not self.stop_event.is_set():
                with self.state_lock:
                    qpos, simtime = self.qpos_snapshot.copy(), self.time_snapshot
                if simtime <= 0:
                    self.stop_event.wait(.01)
                    continue
                local_data.qpos[:] = qpos
                local_data.time = simtime
                self.mujoco.mj_forward(self.model, local_data)
                for row in self.cameras:
                    key = row['instance_id']
                    if last.get(key) == simtime:
                        continue
                    params = row.get('parameters', {})
                    size = (int(params.get('width', 320)), int(params.get('height', 240)))
                    if min(size) < 1 or max(size) > 4096:
                        raise ValueError('SIM_CAMERA_SIZE_UNSUPPORTED')
                    if size not in renderers:
                        renderers[size] = self.mujoco.Renderer(self.model, height=size[1], width=size[0])
                    renderer = renderers[size]
                    camera = move_camera(key, views.get(key) or free_camera(key), size)
                    renderer.update_scene(local_data, camera=camera)
                    pixels = renderer.render()
                    output = io.BytesIO()
                    Image.fromarray(pixels).save(output, format='JPEG', quality=90)
                    now = self.runtime.clock()
                    image = {'data': output.getvalue(), 'encoding': 'jpeg', 'width': size[0], 'height': size[1],
                             'sample_ns': now, 'source_ns': int(simtime * 1_000_000_000), 'source': 'simulation',
                             'synthetic': True, 'measured': False, 'valid': True, 'scene_id': id(self)}
                    sample = {'ready': True, 'sample_ns': now, 'source': 'simulation', 'synthetic': True,
                              'measured': False, 'scene_id': id(self), 'simulation_time_s': simtime,
                              'simulation_level': self.artifact['simulation_level'], 'ports': {'rgb': image}}
                    with self.state_lock:
                        self.camera_cache[key] = sample
                    last[key] = simtime
                fps = max(float(row.get('parameters', {}).get('fps', 30)) for row in self.cameras)
                self.stop_event.wait(1.0 / min(120, max(1, fps)))
        except Exception as exc:
            self.render_error = 'SIM_RENDERING_UNAVAILABLE: ' + str(exc)
        finally:
            for renderer in renderers.values():
                renderer.close()

    def health(self, key):
        with self.state_lock:
            result = copy.deepcopy(self.cache.get(key, self.camera_cache.get(key, {'ready': False, 'sample_ns': 0})))
        if key in self.camera_ids and self.render_error:
            result.update(ready=False, reason=self.render_error, fault=True)
        if self.error or self.closed:
            result.update(ready=False, reason=self.error or 'SIM_CLOSED')
        if 'ports' in result:
            for port in result['ports'].values():
                port.pop('data', None)
        return result

    def read(self, key):
        with self.state_lock:
            return copy.deepcopy(self.cache.get(key, self.camera_cache.get(key, {'ready': False, 'sample_ns': 0, 'ports': {}})))

    def view(self, key, action, dx=0., dy=0.):
        # Camera moves are queued for the render thread: MjvCamera is only
        # touched there, and a drag must never wait on a JPEG encode.
        if key not in self.camera_ids:
            raise KeyError('sim camera not found: ' + str(key))
        if action not in ('orbit', 'pan', 'zoom', 'reset'):
            raise ValueError('unknown camera view action: ' + str(action))
        if not all(type(value) in (int, float) and -5000 <= value <= 5000 for value in (dx, dy)):
            raise ValueError('camera view delta out of range')
        with self.view_lock:
            self.view_actions.setdefault(key, []).append((action, float(dx), float(dy)))
        return {'accepted': True, 'action': action}

    def activate(self, key):
        if not self.configured or self.closed or self.error:
            raise RuntimeError('SIM_NOT_READY')
        with self.command_lock:
            self.active.add(key)
        return True

    def submit(self, key, command):
        with self.command_lock:
            if key not in self.active:
                raise RuntimeError('SIM_DISARMED')
            self.pending[key] = list(command['values'])
        return {'accepted': True, 'status': 'queued_latest', 'execution_ack': False}

    def stop(self, key):
        with self.command_lock:
            self.active.discard(key)
            self.pending.pop(key, None)
            # Explicit hold policy, never a zero-position stop: leave the last
            # command in ctrl. Re-targeting the measured pose instead would
            # hand back the gravity droop and sag a little on every stop.
        return True

    def close(self):
        if self.closed: return
        self.closed = True
        self.stop_event.set()
        if hasattr(self, 'render_thread'):
            self.render_thread.join(timeout=.5)
            if self.render_thread.is_alive():
                raise TimeoutError('SIM_RENDER_THREAD_STUCK')
        if hasattr(self, 'thread'):
            self.thread.join(timeout=.5)
            if self.thread.is_alive():
                raise TimeoutError('SIM_STEP_THREAD_STUCK')


class MujocoAdapter:
    backend = 'sim'
    def __init__(self, instance, session):
        self.instance, self.session, self.active = instance, session, False
        self.key = instance['instance_id']
    def configure(self, context): self.session.configure()
    def health(self, now_ns): return self.session.health(self.key)
    def read(self, now_ns): return self.session.read(self.key)
    def view(self, action, dx=0., dy=0.): return self.session.view(self.key, action, dx, dy)
    def prepare(self): return not self.session.closed
    def activate(self):
        self.active = self.session.activate(self.key)
        return self.active
    def submit(self, command): return self.session.submit(self.key, command)
    def stop(self):
        self.active = False
        return self.session.stop(self.key)
    def close(self): self.session.close()
    def operate(self, operation):
        if operation == 'disable': return self.stop()
        raise RuntimeError('SIM_OPERATION_NOT_IMPLEMENTED: ' + operation)


def make_sim_adapters(runtime):
    selected = [row for row in runtime.instances.values() if row.get('backend') == 'sim']
    if not selected: return {}
    if any(row.get('kind') != 'actuator' and not row['plugin_id'].startswith('morla.camera') for row in selected):
        raise ValueError('SIM_SENSOR_PROVIDER_UNAVAILABLE')
    session = MujocoDeviceSession(runtime)
    return {row['instance_id']: MujocoAdapter(row, session) for row in selected}

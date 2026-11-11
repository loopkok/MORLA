"""Coupled MMK2 scene hosted by the public MORLA runtime.

One scene clock, command slot per component, explicit SI ports. The mobile
base is a planar differential-drive reference, not a traction simulation.
"""
import io
import math
import threading
import time
import xml.etree.ElementTree as E
import numpy as np
from morla_runtime.sim_mujoco import MujocoDeviceSession, MujocoAdapter
from morla_core.resource_locator import resolve_plugin_resource

PREFIX = 'morla.airbot.mmk2.'


def model_xml():
    path = resolve_plugin_resource('mmk2_description', 'mjcf/mmk2.xml')
    root = E.parse(path).getroot()
    root.find('compiler').set('meshdir', str(path.parent.parent/'meshes'))
    return E.tostring(root, encoding='unicode')


class MMK2Session(MujocoDeviceSession):
    def __init__(self, runtime):
        super().__init__(runtime, builder=lambda _: None)
        self.roles = {}
        self.indices, self.targets = {}, {}
        self.camera_ids = {}
        self.base_ctrl = []
        self.command_lock = threading.RLock()
        self.last_base_command_ns = 0
        self.base_timeout_ns = 250_000_000

    def configure(self):
        if self.configured:
            return
        import mujoco
        self.mujoco, self.np = mujoco, np
        self.artifact = {'xml': model_xml(), 'simulation_level':'urdf_planar_reference_contact_unverified'}
        self.model = mujoco.MjModel.from_xml_string(self.artifact['xml'])
        self.data = mujoco.MjData(self.model)
        self.rows = {k:r for k,r in self.runtime.instances.items() if r.get('backend')=='sim'}
        self.indices, self.command_limits, self.targets = {}, {}, {}
        self.roles = {}
        for key, row in self.rows.items():
            if not row['plugin_id'].startswith(PREFIX):
                raise ValueError('MMK2_SCENE_UNSUPPORTED_COMPONENT: '+key)
            role = row['plugin_id'][len(PREFIX):]
            if role not in ('camera','sensors') and role in self.roles:
                raise ValueError('MMK2_SCENE_DUPLICATE_ROLE: '+role)
            if role not in ('camera','sensors'):
                self.roles[role] = key
            if role=='base':
                timeout=row['parameters'].get('command_timeout_s',.25)
                if isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or not .05<=timeout<=1:
                    raise ValueError('MMK2_INVALID_BASE_COMMAND_TIMEOUT')
                self.base_timeout_ns=int(timeout*1e9)
            if row['kind'] != 'actuator':
                continue
            ids = [mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,j['name']) for j in row['joints']]
            if any(i<0 for i in ids):
                raise ValueError('MMK2_JOINT_MAPPING_MISSING')
            controls = [next(i for i in range(self.model.nu) if self.model.actuator_trnid[i,0]==j) for j in ids]
            self.indices[key] = dict(qpos=self.model.jnt_qposadr[ids].tolist(),qvel=self.model.jnt_dofadr[ids].tolist(),ctrl=controls)
            bounds = np.array([[j['lower'],j['upper']] for j in row['joints']])
            self.command_limits[key] = bounds
            initial = np.asarray(row['parameters'].get('initial_positions',np.zeros(len(ids))),dtype=float)
            if initial.shape!=(len(ids),) or not np.isfinite(initial).all() or np.any(initial<bounds[:,0]) or np.any(initial>bounds[:,1]):
                raise ValueError('MMK2_INVALID_INITIAL_POSITION')
            if role=='base' and np.any(initial):
                raise ValueError('MMK2_BASE_INITIAL_VELOCITY_MUST_BE_ZERO')
            self.targets[key] = initial.copy()
            self.data.qpos[self.indices[key]['qpos']] = initial
            self.data.ctrl[controls] = initial
        self.base_qpos = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,'base_'+s)] for s in ('x','y','yaw')]
        self.base_ctrl = [mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_ACTUATOR,'base_'+s+'_drive') for s in ('x','y','yaw')]
        self.cameras = [r for r in self.rows.values() if r['plugin_id']==PREFIX+'camera']
        self.camera_ids = {r['instance_id']:mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,r['parameters']['camera']) for r in self.cameras}
        self.sensor_rows = [r for r in self.rows.values() if r['plugin_id']==PREFIX+'sensors']
        self.last_lidar = np.full(180,10.0); self.last_lidar_time = -1.0
        # Initialize both passive fingers before the first forward/physics step.
        self._mimic()
        mujoco.mj_forward(self.model,self.data)
        self._snapshot()
        self.configured = True
        self.thread = threading.Thread(target=self._step,name='morla-mmk2-scene',daemon=True)
        self.thread.start()
        if self.display in ('local','both'):
            self._open_viewer()
        if self.cameras:
            self.render_thread = threading.Thread(target=self._render,name='morla-mmk2-render',daemon=True)
            self.render_thread.start()

    def _mimic(self):
        for side in ('left','right'):
            a,b = [self.mujoco.mj_name2id(self.model,self.mujoco.mjtObj.mjOBJ_JOINT,side+'_gripper_joint'+suffix) for suffix in ('','1')]
            self.data.qpos[self.model.jnt_qposadr[b]] = self.data.qpos[self.model.jnt_qposadr[a]]

    def _step(self):
        try:
            dt = float(self.model.opt.timestep)
            while not self.stop_event.is_set():
                began = time.monotonic()
                with self.step_lock:
                    with self.command_lock:
                        pending, self.pending = self.pending, {}
                        self.targets.update({key: np.asarray(value) for key, value in pending.items()})
                        base = self.roles.get('base')
                        if base and self.runtime.clock() - self.last_base_command_ns > self.base_timeout_ns:
                            self.targets[base] = np.zeros(2)
                    for key,indices in self.indices.items():
                        self.data.ctrl[indices['ctrl']] = self.targets[key]
                    wheel = self.targets.get(self.roles.get('base'),np.zeros(2))
                    # URDF wheel axes point along -Y: positive joint speed is forward.
                    radius, track = .08401, .3265  # supplied URDF wheel origins
                    v = radius*(wheel[0]+wheel[1])/2
                    w = radius*(wheel[1]-wheel[0])/track
                    yaw = self.data.qpos[self.base_qpos[2]]
                    velocity = np.array([v*math.cos(yaw),v*math.sin(yaw),w])
                    self.data.ctrl[self.base_ctrl] = velocity
                    if self.mode=='kinematic':
                        for key,indices in self.indices.items():
                            if key==self.roles.get('base'):
                                self.data.qpos[indices['qpos']] += self.targets[key]*dt
                            else:
                                self.data.qpos[indices['qpos']] = self.targets[key]
                        self.data.qpos[self.base_qpos] += velocity*dt
                        self.data.qvel[:] = 0
                        self.data.qvel[self.base_qpos] = velocity  # all joints are scalar
                        if 'base' in self.roles:
                            self.data.qvel[self.indices[self.roles['base']]['qvel']] = wheel
                        self._mimic()
                        self.data.time += dt
                        self.mujoco.mj_forward(self.model,self.data)
                    else:
                        self.mujoco.mj_step(self.model,self.data)
                    self._snapshot()
                self.stop_event.wait(max(0,dt-(time.monotonic()-began)))
        except Exception as exc:
            self.error = str(exc)
            self.stop_event.set()

    def _sample(self, ports):
        now = self.runtime.clock()
        return dict(ready=True,sample_ns=now,source='simulation',measured=False,synthetic=True,
                    simulation_time_s=float(self.data.time),simulation_mode=self.mode,
                    simulation_level=self.artifact['simulation_level'],scene_id=id(self),ports={
                        k:dict(values=v,unit=unit,sample_ns=now,source='simulation',valid=True,measured=False,synthetic=True)
                        for k,(v,unit) in ports.items()})

    def _snapshot(self):
        samples = {}
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise ValueError('MMK2_NONFINITE_STATE')
        for key,indices in self.indices.items():
            q = self.data.qpos[indices['qpos']].tolist(); dq = self.data.qvel[indices['qvel']].tolist()
            row = self.rows[key]
            samples[key] = self._sample({'position':(q,row['ports']['position']['unit']),
                                         'velocity':(dq,row['ports']['velocity']['unit'])})
            samples[key]['values'] = q
        if self.sensor_rows:
            x,y,yaw = self.data.qpos[self.base_qpos]
            if self.data.time-self.last_lidar_time >= .1 or self.last_lidar_time<0:
                group = np.array([1,0,0,0,0,0],dtype=np.uint8)  # world geometry only; omit own visual meshes
                origin = np.array([x,y,.25]); geomid = np.zeros(1,dtype=np.int32)
                self.last_lidar = np.array([self.mujoco.mj_ray(self.model,self.data,origin,
                    np.array([math.cos(yaw+a),math.sin(yaw+a),0.]),group,1,-1,geomid)
                    for a in np.linspace(-math.pi,math.pi,180,endpoint=False)])
                self.last_lidar = np.where(self.last_lidar<0,10.,np.minimum(self.last_lidar,10.))
                self.last_lidar_time = float(self.data.time)
                self.lidar_sample_ns = self.runtime.clock()
            for row in self.sensor_rows:
                sample = self._sample({'odometry':([float(x),float(y),0.,0.,0.,math.sin(yaw/2),math.cos(yaw/2)],'m_rad'),
                    'gyro':(self.data.sensor('imu_gyro').data.tolist(),'rad/s'),
                    'acceleration':(self.data.sensor('imu_acc').data.tolist(),'m/s^2'),
                    'lidar':(self.last_lidar.tolist(),'m')})
                sample['ports']['lidar']['sample_ns'] = self.lidar_sample_ns
                samples[row['instance_id']] = sample
        with self.state_lock:
            self.cache = samples
            self.qpos_snapshot = self.data.qpos.copy()
            self.time_snapshot = float(self.data.time)

    def submit(self, key, command):
        row = self.rows[key]
        if not isinstance(command.get('values'),(list,tuple)) or any(isinstance(v,bool) or not isinstance(v,(int,float)) for v in command['values']):
            raise ValueError('MMK2_COMMAND_REQUIRES_NUMERIC_VALUES')
        mode = row['command_modes'][0]
        unit = row['ports'][mode+'_target']['unit']
        if (command.get('names') != [j['name'] for j in row['joints']] or
                command.get('mode') != mode or command.get('units') not in (unit, [unit]*len(row['joints']))):
            raise ValueError('MMK2_COMMAND_NAMES_MODE_OR_UNITS')
        with self.command_lock:
            result=super().submit(key,command)
            if key==self.roles.get('base'):
                self.last_base_command_ns=self.runtime.clock()
            return result

    def stop(self, key):
        if key != self.roles.get('base'):
            return super().stop(key)
        # Serialize zeroing the live controls with any in-flight physics step.
        with self.step_lock, self.command_lock:
            super().stop(key)
            self.targets[key] = np.zeros(2)
            self.data.ctrl[self.indices[key]['ctrl']] = 0
            self.data.ctrl[self.base_ctrl] = 0
        return True

    def set_options(self, mode=None, display=None):
        mode = self.mode if mode is None else mode
        display = self.display if display is None else display
        self._validate_options(mode, display)
        with self.runtime.lock:
            if mode != self.mode and (self.active or self.runtime.status == 'ACTIVE'):
                raise RuntimeError('SIM_DISARM_BEFORE_MODE_CHANGE')
            if display in ('local','both'):
                self._open_viewer()
            elif self.viewer is not None:
                self.viewer.close(); self.viewer = None
            if mode != self.mode:
                with self.step_lock:
                    with self.command_lock:
                        self.pending.clear()
                    for key,indices in self.indices.items():
                        target = np.zeros(2) if key==self.roles.get('base') else self.data.qpos[indices['qpos']].copy()
                        self.targets[key] = target
                        self.data.ctrl[indices['ctrl']] = target
                    self.data.ctrl[self.base_ctrl] = 0
                    self.data.qvel[:] = 0
                    self.mode = mode
                    self._snapshot()
            self.display = display
        return self.settings()

    def _render(self):
        renderers = {}
        try:
            from PIL import Image
            data = self.mujoco.MjData(self.model)
            last = {}
            while not self.stop_event.is_set():
                with self.state_lock:
                    qpos, simtime = self.qpos_snapshot.copy(),self.time_snapshot
                data.qpos[:] = qpos; data.time = simtime
                self.mujoco.mj_forward(self.model,data)
                for row in self.cameras:
                    key = row['instance_id']; p = row['parameters']
                    if simtime-last.get(key,-1) < 1/float(p['fps']):
                        continue
                    size = (int(p['height']),int(p['width']))
                    if min(size)<16 or size[0]>960 or size[1]>1280:
                        raise ValueError('MMK2_CAMERA_SIZE_UNSUPPORTED')
                    if size not in renderers:
                        renderers[size] = self.mujoco.Renderer(self.model,height=size[0],width=size[1])
                    renderer = renderers[size]
                    renderer.disable_depth_rendering()
                    renderer.update_scene(data,camera=self.camera_ids[key])
                    output = io.BytesIO(); Image.fromarray(renderer.render()).save(output,format='JPEG',quality=90)
                    renderer.enable_depth_rendering()
                    renderer.update_scene(data,camera=self.camera_ids[key])
                    depth = np.asarray(renderer.render(),dtype='<f4').tobytes()
                    now = self.runtime.clock()
                    metadata = dict(sample_ns=now,source_ns=int(simtime*1e9),source='simulation',
                                    synthetic=True,measured=False,valid=True,width=size[1],height=size[0],scene_id=id(self))
                    sample = dict(ready=True,sample_ns=now,source='simulation',synthetic=True,measured=False,
                                  simulation_time_s=simtime,scene_id=id(self),ports={
                        'rgb':dict(metadata,data=output.getvalue(),encoding='jpeg'),
                        'depth':dict(metadata,data=depth,encoding='32FC1',unit='m')})
                    with self.state_lock:
                        self.camera_cache[key] = sample
                    last[key] = simtime
                self.stop_event.wait(.005)
        except Exception as exc:
            self.render_error = 'SIM_RENDERING_UNAVAILABLE: '+str(exc)
        finally:
            for renderer in renderers.values():
                renderer.close()

    def view(self,key,action,dx=0.,dy=0.):
        raise ValueError('MMK2_FIXED_CALIBRATION_CAMERA: use local viewer for orbit')


def create_adapters(runtime):
    selected = [r for r in runtime.instances.values() if r.get('backend')=='sim']
    if any(not r['plugin_id'].startswith(PREFIX) for r in selected):
        raise ValueError('MMK2_SCENE_REQUIRES_OWN_COMPONENTS')
    if any(r['kind']=='actuator' and r.get('backend')!='sim' for r in runtime.instances.values()):
        raise ValueError('MMK2_SCENE_REQUIRES_ALL_ACTUATORS_SIM')
    session = MMK2Session(runtime)
    return {r['instance_id']:MujocoAdapter(r,session) for r in selected}

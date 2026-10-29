"""Persistent assembly revisions and a single authoritative composable session."""
from __future__ import annotations
from contextlib import contextmanager
import copy
import fcntl
import json
import os
from pathlib import Path
import re
import threading
import time
import uuid

from morla_core.composition import compile_system
from morla_core.plugins.registry import default_catalog
from morla_runtime import RuntimeSession
from morla_input.adapters import MobileAdapter, InputError
from morla_teleop import BindingController, ControllerRegistry
from morla_learning import TrainingSpec

_ID = re.compile(r"^[a-z][a-z0-9_-]{0,63}$")


class ServiceError(ValueError):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


class AssemblyStore:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    @contextmanager
    def locked(self):
        with (self.root / ".store.lock").open("a") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle, fcntl.LOCK_UN)

    def path(self, assembly_id):
        if not isinstance(assembly_id, str) or not _ID.fullmatch(assembly_id):
            raise ServiceError("invalid assembly id", 422)
        path = self.root / assembly_id
        if path.is_symlink():
            raise ServiceError("assembly symlink is not allowed", 422)
        return path

    def get(self, assembly_id, revision=None):
        path = self.path(assembly_id)
        if revision is None:
            try:
                revision = json.loads((path / "latest.json").read_text())["revision"]
            except FileNotFoundError:
                raise ServiceError("assembly not found", 404)
        if type(revision) is not int or revision < 1:
            raise ServiceError("invalid revision", 422)
        source = path / f"revision-{revision}.json"
        if source.is_symlink():
            raise ServiceError("revision symlink is not allowed", 422)
        try:
            return json.loads(source.read_text())
        except FileNotFoundError:
            raise ServiceError("revision not found", 404)

    def list(self):
        result=[]
        for path in sorted(self.root.iterdir()):
            if not path.is_dir() or not _ID.fullmatch(path.name) or path.is_symlink():
                continue
            item=self.get(path.name)
            result.append({"assembly_id": path.name, "revision": item["assembly"]["revision"],
                           "assembly_hash": item["resolved"]["assembly_hash"]})
        return result

    def save(self, assembly, deployment, resolved, expected_revision):
        assembly_id = assembly["assembly_id"]
        path=self.path(assembly_id)
        with self.locked():
            current=0
            if (path / "latest.json").exists():
                current=self.get(assembly_id)["assembly"]["revision"]
            if type(expected_revision) is not int or current != expected_revision:
                raise ServiceError("assembly revision conflict")
            if assembly.get("revision") != current+1:
                raise ServiceError("next revision must be current + 1", 422)
            path.mkdir(exist_ok=True)
            target=path / f"revision-{current+1}.json"
            value={"assembly": assembly, "deployment": deployment, "resolved": resolved}
            encoded=json.dumps(value,ensure_ascii=False,sort_keys=True,allow_nan=False)
            if target.is_symlink(): raise ServiceError("revision symlink is not allowed",422)
            if target.exists():
                # A process may have crashed after committing a full immutable
                # revision but before publishing latest.json. Recover the same
                # payload, and never overwrite an already committed revision.
                try:
                    previous=json.loads(target.read_text())
                except json.JSONDecodeError:
                    target.rename(path/('.incomplete-'+target.name+'-'+uuid.uuid4().hex))
                else:
                    if json.dumps(previous,sort_keys=True,ensure_ascii=False,allow_nan=False)!=encoded:
                        raise ServiceError("unpublished revision conflict; recover the existing version")
            if not target.exists():
                staged=path/('.revision-'+uuid.uuid4().hex)
                with staged.open("x",encoding="utf-8") as f:
                    f.write(encoded); f.flush(); os.fsync(f.fileno())
                staged.replace(target)
            temporary=path / (".latest-"+uuid.uuid4().hex)
            temporary.write_text(json.dumps({"revision": current+1}))
            temporary.replace(path / "latest.json")
        return copy.deepcopy(value)


class ComposableService:
    def __init__(self, root, *, resource_root=None, blockers=None, clock=time.monotonic_ns):
        self.store=AssemblyStore(Path(root)/"assemblies")
        self.data_root=Path(root)/"recordings"
        self.catalog=default_catalog()
        self.resource_root=resource_root
        self.blockers=blockers or (lambda: [])
        self.clock=clock
        self.lock=threading.RLock()
        self.session=None
        self.requests={}
        self.inputs={}
        self.bindings={}
        self.sequence=0
        self.connections={}
        self.active_input_sources=set()
        self.recorder=None
        self.last_capture_ns=0
        self.wall_offset_ns=time.time_ns()-clock()
        self.ros_inputs=None
        self.capture_source_ns={}
        self.policy=None
        self.policy_error=None
        self.last_policy_ns=0
        self.controller_close_errors=[]
        self.input_errors={}

    def _reset_bindings(self, close=False):
        for controller in self.bindings.values():
            try:
                controller.close() if close else controller.reset()
            except Exception as exc:
                if not close:raise
                self.controller_close_errors.append(str(exc))
        if close:self.bindings={}

    def compile(self, assembly, deployment=None):
        return compile_system(assembly, deployment, registry=self.catalog)

    def save(self, raw):
        with self.lock:
            assembly=raw["assembly"]; deployment=raw.get("deployment")
            resolved=self.compile(assembly,deployment)
            return self.store.save(assembly,deployment,resolved,raw.get("expected_revision",0))

    @property
    def active(self):
        return self.session is not None and (self.session.status!='STOPPED' or bool(self.session.resources.handles))

    def start(self, raw):
        with self.lock:
            request_id=raw.get("request_id")
            if not isinstance(request_id,str) or not request_id or len(request_id)>128:
                raise ServiceError("request_id required",422)
            identity=(raw.get("assembly_id"),raw.get("revision"),raw.get("assembly_hash"))
            if request_id in self.requests:
                previous=self.requests[request_id]
                if previous[0]!=identity:
                    raise ServiceError("request_id reused for different start")
                return previous[1]
            blockers=self.blockers()
            if blockers:
                raise ServiceError("legacy lane active: "+"; ".join(blockers))
            if self.active:
                raise ServiceError("a composable session is already active")
            saved=self.store.get(raw["assembly_id"],raw["revision"])
            resolved=self.compile(saved["assembly"],saved["deployment"])
            if (not raw.get("assembly_hash") or resolved["assembly_hash"]!=raw["assembly_hash"]
                    or json.dumps(resolved,sort_keys=True)!=json.dumps(saved["resolved"],sort_keys=True)):
                raise ServiceError("preview/installed dependency hash changed; compile and save a new revision")
            from morla_input.adapters.ingress import InputIngressAdapter
            adapters={x['instance_id']:InputIngressAdapter(x,self.clock) for x in resolved['instances'] if x['kind']=='input'}
            session=RuntimeSession(resolved,resource_root=self.resource_root,clock=self.clock,registry=self.catalog,adapters=adapters)
            session.start()
            self.session=session
            session.on_command(self.capture_command)
            session.on_event(self.capture_event)
            self._reset_bindings(close=True)
            self.inputs={}; self.bindings={}; self.sequence=0
            self.active_input_sources=set()
            self.input_errors={}
            self.requests[request_id]=(identity,session.state())
            return session.state()

    def require(self, session_id):
        if self.session is None or self.session.session_id!=session_id:
            raise ServiceError("unknown session",404)
        return self.session

    def state(self, session_id):
        with self.lock:
            session=self.require(session_id)
            result=session.state()
            result["inputs"]={key:{"connected":item.connected,"fresh":item.read() is not None,
                                    "epoch":item.epoch,"mode":getattr(item,'mode',None)} for key,item in self.inputs.items()}
            result["recording"] = self.recorder is not None
            result["policy"]={"running":self.policy is not None and not self.policy.closed,"error":self.policy_error}
            result['controller_close_errors']=list(self.controller_close_errors)
            result['input_errors']=dict(self.input_errors)
            return result

    def operation(self, session_id, raw):
        with self.lock:
            session=self.require(session_id)
            request=raw.get("request_id")
            if not isinstance(request,str) or not request:
                raise ServiceError("request_id required",422)
            operation=raw.get("operation")
            if operation=="prepare": return session.prepare(request,raw.get("component_ids"))
            if operation=="activate":
                if request in session.operations or session.status!='READY_DISARMED':
                    return session.activate(request)
                self._reset_bindings()
                # Resolve selected controllers/calibration before any physical enable.
                for binding in session.resolved.get('bindings',[]):
                    controller=self._binding_controller(binding)
                    source=self.inputs.get(binding['source'].split('.',1)[0])
                    if controller.requires_fresh_input and not controller.input_fresh(source.read() if source else None):
                        raise ServiceError('Fresh pose and deadman input required before activation')
                return session.activate(request)
            if operation=="disarm":
                self._reset_bindings()
                self.stop_policy(session_id)
                return session.revoke(reason=operation)
            if operation=="estop":
                self._reset_bindings()
                self.stop_policy(session_id)
                return session.estop()
            if operation=="reset": return session.reset(request)
            if operation=='connect_inputs':
                if self.ros_inputs is not None: return {'status':'SUCCEEDED','already_connected':True}
                from morla_input.ros_sources import RosInputManager
                manager=RosInputManager([x for x in session.instances.values() if x['kind']=='input'],
                    lambda frame:self.ingest_frame(session_id,frame),self.disconnect_input,clock=self.clock)
                try: manager.start()
                except Exception as exc:
                    manager.close()
                    raise ServiceError("INPUT_BACKEND_UNAVAILABLE: "+str(exc)) from exc
                self.ros_inputs=manager
                self.inputs.update(manager.sources)
                return {'status':'SUCCEEDED','sources':list(manager.sources)}
            if operation=="stop":
                self._reset_bindings(close=True)
                self.stop_policy(session_id)
                self.stop_recording()
                if self.ros_inputs is not None:
                    self.ros_inputs.close(); self.ros_inputs=None
                for source in self.inputs.values(): source.disconnect()
                return session.stop(request)
            raise ServiceError("unsupported operation",422)

    def _binding_controller(self, binding):
        controller=self.bindings.get(binding['id'])
        if controller is not None:return controller
        target=binding['target'].split('.',1)[0]
        instance=self.session.instances[target]
        adapter=self.session.adapters[target]
        names=[j['name'] if isinstance(j,dict) else j for j in instance['joints']]
        age=binding.get('max_age_ms',250)
        if isinstance(age,bool) or not isinstance(age,(int,float)) or age<=0:
            raise ServiceError('binding max_age_ms must be positive',422)
        registry=ControllerRegistry()
        if binding['controller'] in {'morla.cartesian_delta_ik','morla.gamepad_cartesian_jog'}:
            from morla_teleop.kinematics import install_kinematics
            install_kinematics(registry,instance)
        elif binding['controller'] in {'morla.xhand.retarget','morla.wuji.retarget'}:
            from morla_teleop.retargeting import install_retargeting
            install_retargeting(registry,instance)
        elif binding['controller'] not in {'morla.joint_jog','morla.scalar_map'}:
            from morla_teleop.retargeting import install_controller_plugin
            install_controller_plugin(registry,binding['controller'],context={
                'instance':instance,'binding':binding,'clock':self.clock,
                'kinematics_provider':getattr(adapter,'kinematics_provider',None)})
        controller=BindingController(binding,names,registry=registry,clock=self.clock,
                                     max_age_ns=int(age*1_000_000))
        self.bindings[binding['id']]=controller
        return controller

    def pair(self, session_id, input_id):
        with self.lock:
            session=self.require(session_id)
            instance=session.instances.get(input_id)
            ports=instance.get('ports',{}) if instance else {}
            if (not instance or instance.get('kind')!='input'
                    or not {'scalar','axes','hold_to_control'}<=set(ports)):
                raise ServiceError("input is not a mobile adapter",422)
            self.disconnect_input(input_id)
            source=MobileAdapter(input_id,clock=self.clock)
            source.reconnect()
            self.inputs[input_id]=source
            return {"input_id":input_id,"token":source.pairing_token,"epoch":source.epoch}

    def connect_mobile(self,session_id,input_id,token):
        with self.lock:
            self.require(session_id)
            source=self.inputs.get(input_id)
            if source is None or not source.authenticate(token):
                raise ServiceError("invalid pairing token",403)
            self.disconnect_input(input_id)
            connection=uuid.uuid4().hex
            source.reconnect()
            self.connections[input_id]=connection
            return source,connection

    def accept_mobile(self, session_id, input_id, raw, connection=None):
        with self.lock:
            session=self.require(session_id)
            if connection is not None and self.connections.get(input_id)!=connection:
                raise ServiceError("retired mobile connection",409)
            source=self.inputs[input_id]
            frame=source.packet(raw)
            return self.ingest_frame(session_id,frame)

    def ingest_frame(self,session_id,frame):
        try:
            return self._ingest_frame(session_id,frame)
        except Exception as exc:
            with self.lock:
                self.input_errors[getattr(frame,'source_id','unknown')]=str(exc)
                self._reset_bindings()
                session=self.require(session_id)
                if session.status=='ACTIVE' and (session.lease is None or session.lease['producer']=='human_mobile'):
                    session.revoke(reason='input_or_controller_failed')
                self.active_input_sources.clear()
            raise

    def _ingest_frame(self,session_id,frame):
        with self.lock:
            session=self.require(session_id)
            input_id=frame.source_id
            if input_id not in session.instances or session.instances[input_id]['kind']!='input':
                raise ServiceError('unregistered input source',422)
            adapter=session.adapters.get(input_id)
            if hasattr(adapter,'feed'): adapter.feed(frame)
            # Disarmed input may be observed, but must never establish an active-motion anchor.
            if session.status!='ACTIVE':
                self._reset_bindings()
                return {'accepted':False,'reason':'explicit_activation_required'}
            command_rows=[]
            for binding in session.resolved.get("bindings",[]):
                if binding["source"].split(".",1)[0]!=input_id:
                    continue
                target=binding["target"].split(".",1)[0]
                instance=session.instances[target]
                joints=instance["joints"]
                names=[j["name"] if isinstance(j,dict) else j for j in joints]
                adapter=session.adapters[target]
                health=adapter.health(self.clock())
                measured=health.get("values")
                if measured is None:
                    raise ServiceError("adapter does not expose measured positions",409)
                controller=self._binding_controller(binding)
                values=controller.step(frame,measured)
                if values is not None:
                    command_rows.append({"component_id":target,"names":names,"values":values,
                                         "mode":"position","units":[j.get("unit","rad") for j in joints]})
            if not command_rows:
                if input_id in self.active_input_sources and session.lease and session.lease["producer"]=="human_mobile":
                    session.revoke(reason="deadman_release")
                    self.active_input_sources.clear()
                return {"accepted":False,"reason":"release_or_reanchor_required"}
            if session.status!="ACTIVE":
                return {"accepted":False,"reason":"explicit_activation_required"}
            lease=session.lease
            if lease is None:
                lease=session.acquire("human_mobile",ttl_ns=250_000_000)
            elif lease["producer"]!="human_mobile":
                raise ServiceError("control authority owned by another producer")
            else:
                lease=session.renew("human_mobile",lease["token"],250_000_000)
            self.sequence+=1
            self.active_input_sources.add(input_id)
            return session.submit({"session_id":session.session_id,"producer":"human_mobile",
                                   "token":lease["token"],"epoch":lease["epoch"],"sequence":self.sequence,
                                   "issued_ns":self.clock(),"ttl_ns":200_000_000,"commands":command_rows})

    def disconnect_input(self, input_id, connection=None):
        with self.lock:
            if connection is not None and self.connections.get(input_id)!=connection:
                return
            self.connections.pop(input_id,None)
            source=self.inputs.get(input_id)
            if source: source.disconnect()
            if (input_id in self.active_input_sources and self.session and self.session.lease
                    and self.session.lease["producer"]=="human_mobile"):
                self.session.revoke(reason="input_disconnect")
                self.active_input_sources.clear()

    def tick(self):
        with self.lock:
            if self.session is None: return
            self.session.tick()
            if (self.session.status=='ACTIVE' and
                    (self.session.lease is None or self.session.lease['producer']=='human_mobile')):
                for binding in self.session.resolved.get('bindings',[]):
                    controller=self.bindings.get(binding['id'])
                    source=self.inputs.get(binding['source'].split('.',1)[0])
                    if (controller and controller.requires_fresh_input and
                            not controller.input_fresh(source.read() if source else None)):
                        self._reset_bindings()
                        self.session.revoke(reason='required_input_expired')
                        self.active_input_sources.clear()
                        break
            if (self.session.lease and self.session.lease["producer"]=="human_mobile"
                    and any(self.inputs.get(key) is None or self.inputs[key].read() is None for key in self.active_input_sources)):
                self.session.revoke(reason="input_expired")
                self.active_input_sources.clear()
            period=int(1_000_000_000/self.session.resolved['capture_schema'].get('dataset_fps',30))
            if self.policy and not self.policy.closed and self.clock()-self.last_policy_ns>=period:
                try:
                    if self.policy.pending is None and not self.policy.actions: self.policy.request()
                    self.policy.poll()
                except Exception as exc:
                    self.policy_error=str(exc)
                    self.policy.close(reason='policy_failed')
                self.last_policy_ns=self.clock()
            if self.recorder and self.clock()-self.last_capture_ns>=period:
                self.capture_tick(); self.last_capture_ns=self.clock()

    def start_policy(self, session_id, raw):
        from morla_learning import ModelManifest
        from morla_policy_inference.composable import RuntimePolicy
        from morla_policy_inference.composable_remote import ManifestRemotePredictor
        with self.lock:
            session=self.require(session_id)
            if self.policy and not self.policy.closed: raise ServiceError('policy already active')
            manifest=ModelManifest.from_dict(raw['manifest'])
            # Serving is local: web requests cannot turn the manager into an arbitrary network client.
            predictor=ManifestRemotePredictor(manifest,host='127.0.0.1',port=8001,prompt=raw.get('prompt',''))
            try:
                policy=RuntimePolicy(session,predictor,manifest,normalization_hash=raw['normalization_hash'],
                    backend_revision=raw.get('backend_revision'),camera_map=raw.get('camera_map'))
                policy.acquire()
            except Exception:
                predictor.close(); raise
            self.policy=policy; self.policy_error=None; self.last_policy_ns=0
            return {'status':'RUNNING','producer':'policy'}

    def stop_policy(self, session_id):
        with self.lock:
            self.require(session_id)
            if self.policy: self.policy.close()
            self.policy=None
            return {'status':'STOPPED'}

    def replay_step(self, session_id, raw):
        from morla_policy_inference.composable import RuntimeReplay
        with self.lock:
            session=self.require(session_id)
            replay=RuntimeReplay(session,raw['frozen_resolved'])
            replay.acquire()
            try: return replay.send(raw['action'],sample_ns=self.clock())
            finally: session.revoke('playback',reason='replay_step_completed')

    def start_recording(self, session_id, task=""):
        with self.lock:
            session=self.require(session_id)
            if self.recorder: raise ServiceError("recording already active")
            from morla_data_collect.composable_recorder import AsyncComposableRecorder as ComposableRecorder
            path=self.data_root/session_id
            recorder=ComposableRecorder(session.resolved["capture_schema"],str(path))
            directory=recorder.start(task=task)
            self.recorder=recorder
            self.capture_source_ns={}
            self.recorder.event('provider_sources',{'resolved_system':session.resolved,
                'runtime_sources':{key:value.health(self.clock()).get('source','unknown') for key,value in session.adapters.items()}})
            self.capture_tick()
            return {"episode_dir":str(directory)}

    def capture_tick(self):
        schema=self.session.resolved["capture_schema"]
        sources=schema.get("metadata",{}).get("stream_sources",{})
        for stream in schema.get("streams",[]):
            source=stream.get("source_kind")
            key=stream["stream_key"]
            mapping=sources.get(key,{})
            instance_id=mapping.get("instance_id",stream.get("instance_id"))
            port_id=mapping.get("port_id")
            if instance_id is None:
                for block in schema.get("state_blocks",[]):
                    if block["stream_key"]==key:
                        instance_id=block.get("instance_id",block["block_id"].split(".")[0])
                        port_id=block["block_id"].split(".",1)[-1]
                        break
            if source=="camera":
                label=key.removeprefix("camera:")
                camera=self.session.resolved.get("feature_contract",{}).get("cameras",{}).get(label,{})
                instance_id=camera.get("instance_id",instance_id)
                port_id=camera.get("port_id",port_id)
            adapter=self.session.adapters.get(instance_id)
            if adapter is None: continue
            sample=adapter.read(self.clock()) if hasattr(adapter,"read") else adapter.health(self.clock())
            port=sample.get("ports",{}).get(port_id,{})
            source_ns=port.get('received_ns',sample.get('sample_ns'))
            if type(source_ns) is not int or not 0<=self.clock()-source_ns<=self.session.max_age_ns: continue
            if source_ns<=self.capture_source_ns.get(key,-1): continue
            timestamp=(self.wall_offset_ns+source_ns)/1e9
            if source=="camera":
                if port.get("data"):
                    self.recorder.append_camera(label,port["data"],timestamp=timestamp)
                    self.capture_source_ns[key]=source_ns
            elif source in {"state","command_echo"}:
                values=port.get("values")
                if values is None and port_id=="position": values=sample.get("values")
                if values is not None:
                    provenance={key:sample.get(key,False) for key in ('source','measured','synthetic')}
                    provenance['source']=sample.get('source','unknown')
                    provenance['command_echo']=sample.get('source')=='command_echo'
                    provenance['valid']=port.get('valid',True)
                    provenance['source_time_ns']=source_ns
                    provenance['received_time_ns']=self.clock()
                    self.recorder.append_stream(key,values,timestamp=timestamp,provenance=provenance)
                    self.capture_source_ns[key]=source_ns
                    for sensor in schema.get('sensor_specs',[]):
                        if sensor.get('source_stream')==key:
                            import numpy as np
                            shape=sensor.get('fields',{}).get('values',{}).get('shape',[len(values)])
                            self.recorder.append_sensor(sensor['sensor_key'],
                                {'values':np.asarray(values).reshape(shape),'valid':np.asarray(bool(provenance['valid']))},timestamp=timestamp)

    def capture_command(self,command,receipt):
        if not self.recorder or not receipt.get("accepted"): return
        targets={x["component_id"]:x for x in command["commands"]}
        for block in self.session.resolved["capture_schema"].get("action_blocks",[]):
            target=targets.get(block.get("writer_id"))
            if target is not None and block.get("command_stream"):
                indices=block.get("stream_indices",list(range(block["dim"])))
                self.recorder.append_stream(block["command_stream"],[target["values"][i] for i in indices],
                                            timestamp=(self.wall_offset_ns+receipt['monotonic_ns'])/1e9,
                                            provenance={'source':'dispatched_command','valid':True,
                                                        'measured':False,'synthetic':False,'command_echo':False})
        self.recorder.event("dispatched",{"producer":command["producer"],"epoch":command["epoch"],
                                         "sequence":command["sequence"],"commands":command["commands"]})

    def capture_event(self,event):
        if self.recorder: self.recorder.event(event["event"],event)

    def stop_recording(self):
        if not self.recorder: return {"status":"not_recording"}
        recorder=self.recorder; self.recorder=None
        return recorder.stop()

    def close(self):
        with self.lock:
            self._reset_bindings(close=True)
            self.stop_recording()
            if self.policy: self.policy.close()
            if self.ros_inputs is not None:
                self.ros_inputs.close(); self.ros_inputs=None
            if self.session and self.session.status!="STOPPED": self.session.stop()

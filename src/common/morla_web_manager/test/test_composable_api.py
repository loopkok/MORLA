import copy
import json
import pytest
from fastapi.testclient import TestClient
from morla_web_manager.composable_api import create_app


def assembly():
    return {"assembly_version":1,"assembly_id":"test_cell","revision":1,
            "instances":{"grip":{"plugin":"morla.astral.gripper","version":"1.0.0"},
                         "phone":{"plugin":"morla.input.mobile","version":"1.0.0"}},
            "bindings":[{"id":"binding","source":"phone.scalar","target":"grip.closure",
                         "deadman":"phone.hold_to_control","controller":"morla.scalar_map","parameters":{}}],
            "capture":{"state":["grip.position"],"action":["grip.closure"],"images":{},"action_source":"command","hold_frames":0}}


def saved_session(client):
    a=assembly()
    saved=client.post('/api/v2/assemblies',json={"assembly":a,"expected_revision":0})
    assert saved.status_code==200,saved.text
    body={"assembly_id":a["assembly_id"],"revision":1,"assembly_hash":saved.json()["resolved"]["assembly_hash"],"request_id":"start-1"}
    started=client.post('/api/v2/sessions',json=body)
    assert started.status_code==200,started.text
    return a,body,started.json()


def test_policy_cannot_start_on_disarmed_devices(tmp_path, monkeypatch):
    from morla_policy_inference import composable_remote
    def unexpected_connection(*args, **kwargs):
        pytest.fail('disarmed policy start must not connect to a predictor')
    monkeypatch.setattr(composable_remote,'ManifestRemotePredictor',unexpected_connection)
    with TestClient(create_app(tmp_path,resource_root=tmp_path/'locks')) as client:
        _,_,state=saved_session(client)
        response=client.post('/api/v2/sessions/'+state['session_id']+'/policy/start',
                             json={'manifest':{},'normalization_hash':'test'})
        assert response.status_code==409
        assert '设备尚未激活' in response.json()['detail']
        assert client.app.state.composable.session.lease is None
        assert client.app.state.composable.policy is None


def test_quest_right_grip_and_web_share_start_and_pause(tmp_path):
    from types import SimpleNamespace
    from morla_input.quest_events import QuestEventSource
    from morla_input.quest_snapshot import QuestSnapshotPublisher
    from morla_input.ros_sources import RosInputManager
    clock = [1_000_000_000]
    with TestClient(create_app(tmp_path, resource_root=tmp_path/'locks')) as client:
        svc = client.app.state.composable
        svc.clock = lambda: clock[0]
        a = assembly()
        a['instances']['phone'] = {'plugin':'morla.input.quest','version':'1.0.0'}
        a['bindings'][0]['deadman'] = 'phone.right_hold'
        saved = client.post('/api/v2/assemblies', json={'assembly':a,'expected_revision':0})
        assert saved.status_code == 200, saved.text
        started = client.post('/api/v2/sessions', json={'assembly_id':a['assembly_id'],
            'revision':1,'assembly_hash':saved.json()['resolved']['assembly_hash'],'request_id':'start'})
        assert started.status_code == 200, started.text
        sid = started.json()['session_id']
        source = QuestEventSource('phone', clock=svc.clock, wall_clock=svc.clock)
        manager = RosInputManager([], lambda frame:svc.ingest_frame(sid,frame),
            lambda iid:svc.disconnect_input(iid), clock=svc.clock,
            on_event=lambda iid,frame,actions:svc.input_event(sid,iid,frame,actions))
        # Propagate callback failures instead of needing a ROS logger in this test.
        manager._guarded = lambda iid,fn:fn()
        manager.sources['phone'] = source
        svc.inputs['phone'] = source
        sequence = [0]
        publisher = QuestSnapshotPublisher()
        levels = {side:{'trigger':0.,'grip':0.,'axes':[0.,0.],'buttons':[0]*6} for side in ('left','right')}
        def send(kind, values, side='right'):
            clock[0] += 10_000_000
            sequence[0] += 1
            if kind in ('trigger','grip'):
                levels[side][kind]=values;kind='controller_state';values=levels[side]
            row=publisher.update(epoch='quest-test',side=side,source='controller',kind=kind,
                values=values,frame_id='quest',basis='unity_left_handed_v1',sent_ns=clock[0])
            manager._quest_event('phone', SimpleNamespace(data=json.dumps(row)))
            import time
            for _ in range(200):
                if svc.operator.handoff.pending is None:break
                svc.tick();time.sleep(.001)
        def op(name):
            response = client.post(f'/api/v2/sessions/{sid}/operations',
                json={'operation':name,'request_id':name+str(clock[0])})
            assert response.status_code == 200, response.text
        send('pose', [0,0,0,0,0,0,1])
        send('trigger', 0.)
        send('grip', 0.)
        send('grip', 1.)
        assert svc.session.status == 'ACTIVE'
        count = sum(e['event']=='activated' for e in svc.session.events)
        send('grip', 1.)
        assert sum(e['event']=='activated' for e in svc.session.events) == count
        op('disarm')
        assert svc.session.status == 'READY_DISARMED'
        send('grip', 1.)
        assert svc.session.status == 'READY_DISARMED'  # Must release before restart.
        send('grip', 0.)
        send('grip', 1.)
        assert svc.session.status == 'ACTIVE'
        send('grip', 0.)
        assert svc.session.status == 'ACTIVE'  # Release is not a global pause.
        op('disarm')
        op('activate')
        assert svc.session.status == 'ACTIVE'


def test_save_conflicts_preview_and_idempotent_launch(tmp_path):
    with TestClient(create_app(tmp_path,resource_root=tmp_path/'locks')) as client:
        a,body,state=saved_session(client)
        assert state['status']=='READY_DISARMED'
        assert client.post('/api/v2/sessions',json=body).json()['session_id']==state['session_id']
        assert client.post('/api/v2/assemblies',json={"assembly":a,"expected_revision":0}).status_code==409
        assert client.post('/api/v2/sessions',json={**body,"request_id":"new","assembly_hash":"wrong"}).status_code==409
        assert client.post('/api/v2/assemblies/compile',json={"assembly":{**a,"shell":"rm -rf"}}).status_code==422
        assert client.post('/api/v2/assemblies',json={"assembly":{**a,"revision":2},"expected_revision":1},headers={"origin":"https://attacker.invalid"}).status_code==403


def test_mobile_input_and_recording(tmp_path):
    with TestClient(create_app(tmp_path,resource_root=tmp_path/'locks')) as client:
        _,_,state=saved_session(client); sid=state['session_id']
        svc=client.app.state.composable
        def op(name): return client.post(f'/api/v2/sessions/{sid}/operations',json={"operation":name,"request_id":name})
        assert op('prepare').json()['status']=='SUCCEEDED'
        assert op('activate').json()['status']=='SUCCEEDED'
        assert client.post(f'/api/v2/sessions/{sid}/recordings',json={"task":"test"}).status_code==200
        pair=client.post(f'/api/v2/sessions/{sid}/inputs/phone/pair',json={}).json()
        with client.websocket_connect(f'/api/v2/sessions/{sid}/inputs/phone/stream') as ws:
            ws.send_json({"token":pair['token']})
            for index,down in enumerate([True,False,True]):
                challenge=ws.receive_json()
                ws.send_json({**challenge,"sequence":index,"hold_to_control":down,"scalar":.7})
                result=ws.receive_json()['receipt']
                # This assembly explicitly binds phone.hold_to_control. The
                # shared engine gates released frames; an absent deadman is
                # the separate continuous-follow case.
                assert result['accepted'] is down, result
        assert svc.session.lease is None
        stopped=client.post(f'/api/v2/sessions/{sid}/recordings/stop',json={})
        assert stopped.status_code==200,stopped.text
        assert svc.session.adapters['grip'].values==[.7]
        assert op('estop').status_code==200
        assert svc.session.status=='ESTOP_LATCHED'


def test_tampered_hash_and_store_path_rejected(tmp_path):
    with TestClient(create_app(tmp_path,resource_root=tmp_path/'locks')) as client:
        a=assembly(); saved=client.post('/api/v2/assemblies',json={"assembly":a}).json()
        assert client.post('/api/v2/sessions',json={"assembly_id":"test_cell","revision":1,"assembly_hash":"wrong","request_id":"bad"}).status_code==409
        assert client.post('/api/v2/assemblies',json={"assembly":{**a,"assembly_id":"../escape"}}).status_code==422


def test_cr5_template_and_camera_preview_contract(tmp_path):
    from types import SimpleNamespace
    with TestClient(create_app(tmp_path, resource_root=tmp_path/'locks')) as client:
        from pathlib import Path
        package=Path(__file__).resolve().parents[3]/'robots/dobot/morla_cr5_driver'
        for filename in ('descriptor.json','controller.json','jog_controller.json'):
            descriptor=json.loads((package/filename).read_text())
            if not any(d['plugin_id']==descriptor['plugin_id'] for d in client.app.state.composable.catalog.list()):
                client.app.state.composable.catalog.register(descriptor)

        template = client.get('/api/v2/templates/cr5-sim')
        assert template.status_code == 200
        assert template.json()['instances']['arm']['backend'] == 'sim'
        _, _, state = saved_session(client)
        sid = state['session_id']
        current = client.get(f'/api/v2/sessions/{sid}/state').json()
        assert current['assembly_id'] == 'test_cell' and current['revision'] == 1
        assert client.get(f'/api/v2/sessions/{sid}/cameras/grip/frame').status_code == 404
        svc = client.app.state.composable
        svc.session.instances['camera'] = {'plugin_id':'example.camera','kind':'sensor','ports':{'rgb':{'semantic':'image'}}}
        svc.session.adapters['camera'] = SimpleNamespace(read=lambda now: {'ports':{}})
        assert client.get(f'/api/v2/sessions/{sid}/cameras/camera/frame').status_code == 503
        svc.session.adapters['camera'].read = lambda now: {'ports':{'rgb':{'valid':True,'data':b'jpeg-bytes','encoding':'jpeg'}}}
        response = client.get(f'/api/v2/sessions/{sid}/cameras/camera/frame')
        assert response.status_code == 200 and response.content == b'jpeg-bytes'
        assert response.headers['cache-control'] == 'no-store'
        # A preview without a viewpoint (real camera) cannot be orbited.
        view = f'/api/v2/sessions/{sid}/cameras/camera/view'
        assert client.post(view, json={'action': 'orbit', 'dx': 10, 'dy': 0}).status_code == 400
        calls = []
        svc.session.adapters['camera'].view = lambda action, dx=0., dy=0.: calls.append((action, dx, dy)) or {'accepted': True}
        assert client.post(view, json={'action': 'orbit', 'dx': 10, 'dy': -4}).json() == {'accepted': True}
        assert client.post(view, json={'action': 'reset'}).status_code == 200
        assert calls == [('orbit', 10., -4.), ('reset', 0., 0.)]
        assert client.post(view, json={'action': 'spin'}).status_code == 422
        assert client.post(view, json={'action': 'zoom', 'dx': 0, 'dy': 1e9}).status_code == 422
        assert client.post(f'/api/v2/sessions/{sid}/cameras/grip/view', json={'action': 'orbit'}).status_code == 404
        svc.session.adapters.pop('camera'); svc.session.instances.pop('camera')


def test_shared_simulation_templates_and_live_settings(tmp_path, monkeypatch):
    pytest.importorskip('mujoco')
    with TestClient(create_app(tmp_path,resource_root=tmp_path/'locks')) as client:
        from pathlib import Path
        package=Path(__file__).resolve().parents[3]/'robots/dobot/morla_cr5_driver'
        for filename in ('descriptor.json','controller.json','jog_controller.json'):
            descriptor=json.loads((package/filename).read_text())
            if not any(d['plugin_id']==descriptor['plugin_id'] for d in client.app.state.composable.catalog.list()):
                client.app.state.composable.catalog.register(descriptor)

        for robot in ('cr5','nero','astral'):
            response=client.get(f'/api/v2/templates/{robot}-sim')
            assert response.status_code==200
            assert response.json()['metadata']['simulation']['mode']=='kinematic'
        assert client.get('/api/v2/templates/unknown').status_code==404
        a={'assembly_version':1,'assembly_id':'sim_options','revision':1,
           'instances':{'arm':{'plugin':'morla.nero.arm','backend':'sim'}},
           'metadata':{'simulation':{'mode':'kinematic','display':'web'}}}
        saved=client.post('/api/v2/assemblies',json={'assembly':a,'expected_revision':0})
        assert saved.status_code==200,saved.text
        result=client.post('/api/v2/sessions',json={'assembly_id':a['assembly_id'],'revision':1,
            'assembly_hash':saved.json()['resolved']['assembly_hash'],'request_id':'start'})
        assert result.status_code==200,result.text
        sid=result.json()['session_id'];path=f'/api/v2/sessions/{sid}/simulation'
        assert client.post(path,json={'mode':'physics'}).json()['mode']=='physics'
        assert client.post(path,json={'mode':'invalid'}).status_code==422
        assert client.post(path,json={'unexpected':True}).status_code==422
        service=client.app.state.composable;adapter=service.session.adapters['arm']
        adapter.activate()
        assert client.post(path,json={'mode':'kinematic'}).status_code==409
        adapter.stop()
        assert client.post(path,json={'mode':'kinematic'}).status_code==200
        assert client.get(f'/api/v2/sessions/{sid}/state').json()['simulation']['mode']=='kinematic'


def test_diagnostic_logs_are_independent_downloadable_and_confined(tmp_path,monkeypatch):
    from morla_input.adapters.normalized import InputFrame
    monkeypatch.setenv('MORLA_DIAGNOSTIC_LOG_ROOT',str(tmp_path/'logs'))
    with TestClient(create_app(tmp_path/'store',resource_root=tmp_path/'locks')) as client:
        _,_,state=saved_session(client);sid=state['session_id'];service=client.app.state.composable
        assert service.recorder is None
        assert state['diagnostic_log']['enabled'] and state['diagnostic_log']['directory']
        service.operation(sid,{'operation':'prepare','request_id':'prepare-log'})
        service.operation(sid,{'operation':'activate','request_id':'activate-log'})
        frame=InputFrame('phone','epoch',0,service.clock(),'operator',{'scalar':.7,'hold_to_control':True})
        assert service.ingest_frame(sid,frame)['accepted']
        assert service.recorder is None
        active_download=client.get(f'/api/v2/diagnostics/runs/{sid}/files/trace-00000.jsonl')
        assert active_download.status_code==200
        assert int(active_download.headers['content-length'])==len(active_download.content)
        service.operation(sid,{'operation':'stop','request_id':'stop-log'})
        runs=client.get('/api/v2/diagnostics/runs').json();entry=next(row for row in runs if row['session_id']==sid)
        assert entry['clean_shutdown'] and entry['state']=='closed'
        assert {'manifest.json','summary.json','trace-00000.jsonl'}<=set(entry['files'])
        tail=client.get(f'/api/v2/diagnostics/runs/{sid}/tail?limit=200')
        assert tail.status_code==200
        assert {'input','control','command_request','command_receipt'}<={row['kind'] for row in tail.json()['records']}
        manifest=client.get(f'/api/v2/diagnostics/runs/{sid}/files/manifest.json')
        assert manifest.status_code==200 and manifest.json()['session_id']==sid
        data=client.get(f'/api/v2/diagnostics/runs/{sid}/files/trace-00000.jsonl')
        assert data.status_code==200 and '[redacted]' in data.text
        assert client.get(f'/api/v2/diagnostics/runs/{sid}/files/other.json').status_code==422
        assert client.get(f'/api/v2/diagnostics/runs/{sid}/tail?limit=10000').status_code==422
        assert client.get('/api/v2/diagnostics/runs/not-a-run/tail').status_code==422
        assert client.get('/api/v2/diagnostics/runs/'+'a'*32+'/files/manifest.json').status_code==404
        directory=service.session.diagnostics.directory
        (directory/'trace-99999.jsonl').symlink_to(tmp_path/'outside')
        assert client.get(f'/api/v2/diagnostics/runs/{sid}/files/trace-99999.jsonl').status_code==404


def test_device_unreachable_is_503_and_unknown_crash_is_json_500(tmp_path):
    """A dead socket must not surface as a bare 500/plain text the UI cannot parse."""
    with TestClient(create_app(tmp_path, resource_root=tmp_path/'locks')) as client:
        store = client.app.state.composable.store

        def unreachable(): raise ConnectionError('CR5 192.168.5.1:29999 unreachable: timed out')
        store.list = unreachable
        response = client.get('/api/v2/assemblies')
        assert response.status_code == 503, response.text
        assert response.headers['content-type'].startswith('application/json')
        assert '192.168.5.1:29999' in response.json()['detail']

        def crash(): raise ZeroDivisionError('division by zero')
        store.list = crash
        response = client.get('/api/v2/assemblies')
        assert response.status_code == 500, response.text
        assert response.headers['content-type'].startswith('application/json')
        assert 'ZeroDivisionError' in response.json()['detail']

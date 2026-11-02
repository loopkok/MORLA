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
                # Deadman gating was removed in the rollback: every frame is
                # forwarded while the session is ACTIVE.
                assert result['accepted'],result
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
        template = client.get('/api/v2/templates/cr5-sim')
        assert template.status_code == 200
        assert template.json()['instances']['arm']['backend'] == 'sim'
        _, _, state = saved_session(client)
        sid = state['session_id']
        current = client.get(f'/api/v2/sessions/{sid}/state').json()
        assert current['assembly_id'] == 'test_cell' and current['revision'] == 1
        assert client.get(f'/api/v2/sessions/{sid}/cameras/grip/frame').status_code == 404
        svc = client.app.state.composable
        svc.session.instances['camera'] = {'plugin_id':'morla.camera'}
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

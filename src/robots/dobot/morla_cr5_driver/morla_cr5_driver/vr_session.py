"""Start a disarmed Quest/CR5 composable session and its local control UI."""
import argparse
import json
import os
import uuid
from pathlib import Path


def attach_control_page(app, service, session_id):
    from fastapi.responses import FileResponse

    @app.get('/')
    def page():
        return FileResponse(Path(__file__).with_name('static')/'cr5_vr.html')

    @app.get('/cr5-vr/session')
    def bootstrap():
        session=service.require(session_id)
        return {'session_id':session_id,'backends':{key:i['backend'] for key,i in session.instances.items()}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assembly',required=True)
    parser.add_argument('--deployment')
    parser.add_argument('--port',type=int,default=8080)
    parser.add_argument('--store',default=os.path.expanduser('~/.local/share/morla-cr5-vr'))
    args=parser.parse_args()
    from morla_core.composition import load_document
    from morla_web_manager.composable_api import create_app
    import uvicorn
    assembly=load_document(args.assembly)
    deployment=load_document(args.deployment) if args.deployment else {}
    app=create_app(os.path.expanduser(args.store),mount_web=False)
    service=app.state.composable
    try:
        existing=service.store.path(assembly['assembly_id'])/'latest.json'
        revision=service.store.get(assembly['assembly_id'])['assembly']['revision'] if existing.exists() else 0
        assembly['revision']=revision+1
        saved=service.save({'assembly':assembly,'deployment':deployment,'expected_revision':revision})
        state=service.start({'assembly_id':assembly['assembly_id'],'revision':assembly['revision'],
            'assembly_hash':saved['resolved']['assembly_hash'],'request_id':uuid.uuid4().hex})
        sid=state['session_id']
        service.operation(sid,{'operation':'connect_inputs','request_id':uuid.uuid4().hex})
        attach_control_page(app,service,sid)
        print(json.dumps({'url':f'http://127.0.0.1:{args.port}','session_id':sid,
                          'status':service.session.status,'activation':'explicit_only'},ensure_ascii=False),flush=True)
        uvicorn.run(app,host='127.0.0.1',port=args.port)
    finally:
        service.close()

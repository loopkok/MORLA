"""Shared composable session host; device launches supply configuration only."""
import argparse
import json
import os
import uuid


def main(args=None, *, configure_app=None, default_store='~/.local/share/morla-composable'):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assembly',required=True);parser.add_argument('--deployment',default='')
    parser.add_argument('--input-device',default=None)
    parser.add_argument('--input-backend',choices=['fake','real'],default=None)
    parser.add_argument('--port',type=int,default=8080)
    parser.add_argument('--store',default=default_store)
    options=parser.parse_args(args)
    from morla_core.composition import load_document
    from .composable_api import create_app
    import uvicorn
    assembly=load_document(options.assembly)
    from morla_core.composition import default_catalog
    catalog=default_catalog()
    for instance in assembly['instances'].values():
        if catalog.get(instance['plugin'],instance.get('version','1.0.0'))['kind']=='input':
            if options.input_backend is not None:instance['backend']=options.input_backend
            if options.input_device:instance.setdefault('parameters',{})['device']=options.input_device
    deployment=load_document(options.deployment) if options.deployment else {}
    app=create_app(os.path.expanduser(options.store),mount_web=configure_app is None)
    service=app.state.composable
    try:
        path=service.store.path(assembly['assembly_id'])/'latest.json'
        revision=service.store.get(assembly['assembly_id'])['assembly']['revision'] if path.exists() else 0
        assembly['revision']=revision+1
        saved=service.save({'assembly':assembly,'deployment':deployment,'expected_revision':revision})
        state=service.start({'assembly_id':assembly['assembly_id'],'revision':assembly['revision'],
            'assembly_hash':saved['resolved']['assembly_hash'],'request_id':uuid.uuid4().hex})
        sid=state['session_id']
        service.operation(sid,{'operation':'connect_inputs','request_id':uuid.uuid4().hex})
        if configure_app is not None:configure_app(app,service,sid)
        print(json.dumps({'url':f'http://127.0.0.1:{options.port}','session_id':sid,
                          'status':service.session.status,'activation':'explicit_only'},ensure_ascii=False),flush=True)
        uvicorn.run(app,host='127.0.0.1',port=options.port)
    finally:service.close()

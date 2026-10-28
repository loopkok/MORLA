"""Bound untrusted configuration bodies before FastAPI parses their fields."""
import asyncio
import json
from urllib.parse import urlsplit
from starlette.responses import JSONResponse


class ConfigurationBoundary:
    def __init__(self,app,max_bytes=1_000_000):
        self.app,self.max_bytes=app,max_bytes

    async def __call__(self,scope,receive,send):
        if scope['type']!='http' or not scope.get('path','').startswith('/api/v2') or scope['method'] not in {'POST','PUT','PATCH','DELETE'}:
            return await self.app(scope,receive,send)
        headers={key.decode().lower():value.decode() for key,value in scope['headers']}
        origin=headers.get('origin')
        if origin and urlsplit(origin).netloc!=headers.get('host'):
            return await JSONResponse({'detail':'cross-origin mutation rejected'},status_code=403)(scope,receive,send)
        chunks=[];count=0
        try:
            while True:
                message=await asyncio.wait_for(receive(),5)
                if message['type']=='http.disconnect':return
                chunk=message.get('body',b'');count+=len(chunk)
                if count>self.max_bytes:
                    return await JSONResponse({'detail':'configuration body too large'},status_code=413)(scope,receive,send)
                chunks.append(chunk)
                if not message.get('more_body'):break
            body=b''.join(chunks)
            def pairs(rows):
                result={}
                for key,value in rows:
                    if key in result: raise ValueError('duplicate JSON key: '+key)
                    result[key]=value
                return result
            if body:
                value=json.loads(body,object_pairs_hook=pairs,parse_constant=lambda value:(_ for _ in ()).throw(ValueError('nonfinite JSON constant')))
                from morla_core.composition.parser import finite_json
                finite_json(value)
        except (ValueError,RecursionError,UnicodeError,asyncio.TimeoutError) as exc:
            return await JSONResponse({'detail':'invalid configuration JSON: '+str(exc)},status_code=422)(scope,receive,send)
        sent=False
        async def replay():
            nonlocal sent
            if not sent:
                sent=True;return {'type':'http.request','body':body,'more_body':False}
            return await receive()
        return await self.app(scope,replay,send)

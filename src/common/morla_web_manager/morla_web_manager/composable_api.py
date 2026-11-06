"""Composable control plane, usable standalone without importing rclpy."""
from __future__ import annotations
import asyncio
from contextlib import asynccontextmanager
import json
import logging
import math
import os
from pathlib import Path
import secrets
from urllib.parse import urlsplit

from fastapi import APIRouter, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse

from .composable_service import ComposableService, ServiceError


def router(service):
    api=APIRouter(prefix="/api/v2")
    def call(fn,*args,**kwargs):
        try: return fn(*args,**kwargs)
        except HTTPException: raise
        except ServiceError as exc: raise HTTPException(exc.status,str(exc)) from exc
        except (ValueError,KeyError,TypeError) as exc:
            raise HTTPException(422,detail=getattr(exc,"diagnostics",str(exc))) from exc
        except FileNotFoundError as exc: raise HTTPException(404,str(exc)) from exc
        except RuntimeError as exc: raise HTTPException(409,str(exc)) from exc
        # OSError covers ConnectionError/TimeoutError: a device, port or socket
        # is unreachable. That is availability, not a bad request, so 503 and a
        # readable reason instead of an unhandled crash the browser cannot parse.
        except OSError as exc:
            raise HTTPException(503,detail=f'{type(exc).__name__}: {exc}') from exc
        except Exception as exc:
            # The operator gets a readable JSON reason; the traceback stays here.
            logging.getLogger('morla.composable').exception('unhandled %s', getattr(fn,'__qualname__',fn))
            raise HTTPException(500,detail=f'{type(exc).__name__}: {exc}') from exc

    @api.get('/active-session')
    def active_session():
        return None if service.session is None else call(service.state,service.session.session_id)

    @api.get("/components")
    def components(): return service.catalog.to_dict()

    @api.get('/templates')
    def device_templates():
        from .templates import list_templates
        return call(list_templates, service.catalog)

    @api.get('/templates/{template_id}')
    def device_template(template_id: str):
        from .templates import load_template
        return call(load_template, service.catalog, template_id)

    @api.post('/sessions/{session_id}/simulation')
    def simulation_options(session_id: str, raw: dict):
        with service.lock:
            session = call(service.require, session_id)
            scenes = {id(adapter.session): adapter.session for adapter in session.adapters.values()
                      if getattr(adapter, 'backend', None) == 'sim' and hasattr(adapter, 'session')}
            if len(scenes) != 1:
                raise HTTPException(409, 'session has no shared MuJoCo scene')
            if set(raw) - {'mode', 'display'}:
                raise HTTPException(422, 'unknown simulation option')
            if service.recorder is not None or service.policy is not None:
                raise HTTPException(409, 'stop recording/policy before changing simulation options')
            settings = call(next(iter(scenes.values())).set_options, **raw)
            session._event('simulation_options', **settings)
            return settings

    @api.get('/sessions/{session_id}/cameras/{camera_id}/frame')
    def camera_frame(session_id: str, camera_id: str):
        session = call(service.require, session_id)
        adapter = session.adapters.get(camera_id)
        instance = session.instances.get(camera_id, {})
        if adapter is None or instance.get('ports',{}).get('rgb',{}).get('semantic') != 'image':
            raise HTTPException(404, 'camera not found')
        sample = adapter.read(session.clock())
        port = sample.get('ports', {}).get('rgb', {})
        if not port.get('valid') or not port.get('data') or port.get('encoding') != 'jpeg':
            raise HTTPException(503, 'camera frame unavailable')
        return Response(port['data'], media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @api.post('/sessions/{session_id}/cameras/{camera_id}/view')
    def camera_view(session_id: str, camera_id: str, raw: dict):
        # Free-camera orbit/pan/zoom for the MuJoCo preview, driven by pointer
        # drags in the page. Only simulated cameras have a viewpoint to move;
        # real camera adapters have no such method and answer 400.
        session = call(service.require, session_id)
        adapter = session.adapters.get(camera_id)
        instance = session.instances.get(camera_id, {})
        if adapter is None or instance.get('ports',{}).get('rgb',{}).get('semantic') != 'image':
            raise HTTPException(404, 'camera not found')
        if not hasattr(adapter, 'view'):
            raise HTTPException(400, 'camera view is not adjustable')
        action = raw.get('action')
        if action not in ('orbit', 'pan', 'zoom', 'reset'):
            raise HTTPException(422, 'invalid camera view action')
        for name in ('dx', 'dy'):
            value = raw.get(name, 0.)
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 5000:
                raise HTTPException(422, 'invalid camera view delta')
        return call(adapter.view, action, float(raw.get('dx', 0.)), float(raw.get('dy', 0.)))

    @api.get('/sessions/{session_id}/cameras/{camera_id}/stream')
    def camera_stream(session_id: str, camera_id: str):
        # Polling one JPEG per request capped the preview at the page's request
        # interval (~10 fps), which dominated the perceived teleop lag. Push the
        # latest cached frame instead: one connection, render-rate frames.
        session = call(service.require, session_id)
        adapter = session.adapters.get(camera_id)
        instance = session.instances.get(camera_id, {})
        if adapter is None or instance.get('ports',{}).get('rgb',{}).get('semantic') != 'image':
            raise HTTPException(404, 'camera not found')

        async def frames():
            last = None
            try:
                while service.session is session:
                    try:
                        sample = adapter.read(session.clock())
                    except Exception:
                        return  # adapter closed with the session
                    port = sample.get('ports', {}).get('rgb', {})
                    data = port.get('data')
                    key = (port.get('source_ns'), port.get('sample_ns'))
                    if port.get('valid') and data and port.get('encoding') == 'jpeg' and key != last:
                        last = key
                        yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                               + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
                    # Poll at 60 Hz: a 1/30 s sleep capped the preview at ~30 fps
                    # even when the camera renders faster, which dominated the
                    # perceived teleop lag.
                    await asyncio.sleep(1 / 60)
            except (asyncio.CancelledError, GeneratorExit):
                return

        return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame',
                                 headers={'Cache-Control': 'no-store'})

    @api.get('/diagnostics/runs')
    def diagnostic_runs():
        from .diagnostics import runs
        return call(runs,service.diagnostic_root,service.session)

    @api.get('/diagnostics/runs/{session_id}/tail')
    def diagnostic_tail(session_id:str,limit:int=50):
        from .diagnostics import tail
        return call(tail,service.diagnostic_root,session_id,limit)

    @api.get('/diagnostics/runs/{session_id}/files/{filename}')
    def diagnostic_file(session_id:str,filename:str):
        from .diagnostics import file_path
        path=call(file_path,service.diagnostic_root,session_id,filename)
        if filename.endswith('.jsonl'):
            # Active segments can grow during download. Freeze the byte length
            # rather than letting FileResponse read past Content-Length.
            length=path.stat().st_size
            def chunks():
                remaining=length
                with path.open('rb') as stream:
                    while remaining:
                        block=stream.read(min(65536,remaining))
                        if not block:break
                        remaining-=len(block)
                        yield block
            return StreamingResponse(chunks(),media_type='application/x-ndjson',
                headers={'Content-Length':str(length),'Content-Disposition':f'attachment; filename="{filename}"','Cache-Control':'no-store'})
        return FileResponse(path,media_type='application/json',filename=filename)

    @api.get("/assemblies")
    def assemblies(): return call(service.store.list)

    @api.post("/assemblies/validate")
    @api.post("/assemblies/compile")
    def compile_assembly(raw:dict):
        return call(service.compile,raw.get("assembly",raw),raw.get("deployment"))

    @api.post("/assemblies")
    def save_assembly(raw:dict): return call(service.save,raw)

    @api.get("/assemblies/{assembly_id}/revisions/{revision}")
    def revision(assembly_id:str,revision:int): return call(service.store.get,assembly_id,revision)

    @api.post("/deployments/preflight")
    def preflight(raw:dict):
        resolved=call(service.compile,raw["assembly"],raw.get("deployment"))
        from morla_runtime import RuntimeSession
        runtime=RuntimeSession(resolved,resource_root=service.resource_root,registry=service.catalog)
        return {"compile_valid":True,"device_probe_performed":False,"preflight":runtime.preflight(),
                "status":resolved['status'],"resolved":resolved}

    @api.post("/sessions")
    def session(raw:dict): return call(service.start,raw)

    @api.get("/sessions/{session_id}/state")
    def state(session_id:str): return call(service.state,session_id)

    @api.post("/sessions/{session_id}/operations")
    def operation(session_id:str,raw:dict): return call(service.operation,session_id,raw)

    @api.get('/sessions/{session_id}/operations/{request_id}')
    def operation_state(session_id:str,request_id:str):
        session=call(service.require,session_id)
        with session.lock:
            if request_id not in session.operations: raise HTTPException(404,'operation not found')
            from copy import deepcopy
            return deepcopy(session.operations[request_id])

    @api.post("/sessions/{session_id}/inputs/{input_id}/pair")
    def pair(session_id:str,input_id:str): return call(service.pair,session_id,input_id)

    @api.post("/sessions/{session_id}/recordings")
    def record(session_id:str,raw:dict): return call(service.start_recording,session_id,raw.get("task",""))

    @api.post("/sessions/{session_id}/recordings/stop")
    def stop_record(session_id:str):
        call(service.require,session_id)
        return call(service.stop_recording)

    @api.post('/sessions/{session_id}/policy/start')
    def policy_start(session_id:str,raw:dict): return call(service.start_policy,session_id,raw)

    @api.post('/sessions/{session_id}/operator/action')
    def operator_action(session_id:str,raw:dict): return call(service.operator_action,session_id,raw)

    @api.post('/sessions/{session_id}/operator/hold')
    def operator_hold(session_id:str,raw:dict): return call(service.operator_hold,session_id,raw)

    @api.post('/sessions/{session_id}/replay/start')
    def replay_start(session_id:str,raw:dict):
        with service.lock:
            call(service.require,session_id)
            return call(service.operator.start_playback,raw)

    @api.post('/sessions/{session_id}/policy/stop')
    def policy_stop(session_id:str): return call(service.stop_policy,session_id)

    @api.post('/sessions/{session_id}/replay/step')
    def replay_step(session_id:str,raw:dict): return call(service.replay_step,session_id,raw)

    @api.post("/training/validate")
    @api.post("/training/configs")
    def training(raw:dict):
        from morla_learning import TrainingSpec
        return call(lambda:TrainingSpec.from_dict(raw).config())

    @api.websocket("/sessions/{session_id}/events")
    async def events(ws:WebSocket,session_id:str):
        await ws.accept()
        try:
            while True:
                await ws.send_json(call(service.state,session_id))
                await asyncio.sleep(.1)
        except (WebSocketDisconnect,HTTPException): pass

    @api.websocket("/sessions/{session_id}/inputs/{input_id}/stream")
    async def mobile(ws:WebSocket,session_id:str,input_id:str):
        origin=ws.headers.get("origin")
        if origin and urlsplit(origin).netloc!=ws.headers.get("host"):
            await ws.close(code=1008); return
        await ws.accept()
        connection=None
        try:
            service.require(session_id)
            text=await asyncio.wait_for(ws.receive_text(),5)
            if len(text)>2048: raise ValueError("authentication payload too large")
            auth=json.loads(text)
            source,connection=service.connect_mobile(session_id,input_id,auth.get("token"))
            while True:
                challenge=secrets.token_hex(12)
                sent=service.clock()
                await ws.send_json({"epoch":source.epoch,"challenge":challenge})
                text=await asyncio.wait_for(ws.receive_text(),.5)
                if len(text)>8192: raise ValueError("input packet too large")
                raw=json.loads(text)
                if raw.pop("challenge",None)!=challenge or service.clock()-sent>250_000_000:
                    raise ValueError("stale input challenge")
                result=service.accept_mobile(session_id,input_id,raw,connection)
                await ws.send_json({"receipt":result})
                await asyncio.sleep(.02)
        except (WebSocketDisconnect,ValueError,KeyError,RuntimeError,asyncio.TimeoutError):
            try: await ws.close(code=1008)
            except RuntimeError: pass
        finally:
            if connection is not None: service.disconnect_input(input_id,connection)
    return api


def create_app(root=None,resource_root=None,*,mount_web=True,diagnostic_root=None):
    service=ComposableService(root or os.environ.get("MORLA_SYSTEM_STORE",str(Path.home()/".local/share/morla")),
                              resource_root=resource_root,diagnostic_root=diagnostic_root)
    @asynccontextmanager
    async def lifespan(app):
        async def tick():
            while True:
                await asyncio.to_thread(service.tick)
                await asyncio.sleep(.02)
        task=asyncio.create_task(tick())
        try: yield
        finally:
            task.cancel()
            try: await task
            except asyncio.CancelledError: pass
            service.close()
    app=FastAPI(title="MORLA Composable Manager",lifespan=lifespan)
    from .request_boundary import ConfigurationBoundary
    app.add_middleware(ConfigurationBoundary)
    app.state.composable=service
    app.include_router(router(service))
    # Starlette's default 500 body is plain text; the browser then reports
    # "JSON.parse: unexpected character" and the operator learns nothing.
    @app.exception_handler(Exception)
    async def unhandled(request:Request,exc:Exception):
        return JSONResponse({'detail':f'{type(exc).__name__}: {exc}'},status_code=500)
    @app.middleware("http")
    async def boundary(request:Request,call_next):
        if request.method in {"POST","PUT","PATCH","DELETE"}:
            origin=request.headers.get("origin")
            if origin and urlsplit(origin).netloc!=request.headers.get("host"):
                return JSONResponse({"detail":"cross-origin mutation rejected"},status_code=403)
            length=request.headers.get("content-length","0")
            if not length.isdigit() or int(length)>1_000_000:
                return JSONResponse({"detail":"request too large"},status_code=413)
        return await call_next(request)
    @app.get("/mobile")
    def mobile_page():
        return FileResponse(Path(__file__).with_name("static")/"mobile.html")
    # Serve the same React build as the existing manager when installed.
    from fastapi.staticfiles import StaticFiles
    import inspect
    dist=Path(__file__).parents[1]/"web/dist"
    if not dist.is_dir():
        try:
            from ament_index_python.packages import get_package_share_directory
            dist=Path(get_package_share_directory("morla_web_manager"))/"web/dist"
        except ImportError: pass
    if mount_web and dist.is_dir():
        options={"directory":dist,"html":True}
        # colcon symlink installs leave the files as symlinks into the source
        # tree; newer Starlette rejects those by default, so every asset would
        # 404 while the build is present. Older versions always followed them.
        if "follow_symlink" in inspect.signature(StaticFiles).parameters:
            options["follow_symlink"]=True
        app.mount("/",StaticFiles(**options),name="web")
    return app


def main():
    import uvicorn
    uvicorn.run(create_app(),host=os.environ.get("MORLA_WEB_MANAGER_HOST","127.0.0.1"),
                port=int(os.environ.get("MORLA_WEB_MANAGER_PORT","8080")))

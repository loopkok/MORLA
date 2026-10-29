"""Composable control plane, usable standalone without importing rclpy."""
from __future__ import annotations
import asyncio
from contextlib import asynccontextmanager
import json
import os
from pathlib import Path
import secrets
from urllib.parse import urlsplit

from fastapi import APIRouter, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

from .composable_service import ComposableService, ServiceError


def router(service):
    api=APIRouter(prefix="/api/v2")
    def call(fn,*args,**kwargs):
        try: return fn(*args,**kwargs)
        except ServiceError as exc: raise HTTPException(exc.status,str(exc)) from exc
        except (ValueError,KeyError,TypeError) as exc:
            raise HTTPException(422,detail=getattr(exc,"diagnostics",str(exc))) from exc
        except RuntimeError as exc: raise HTTPException(409,str(exc)) from exc

    @api.get("/components")
    def components(): return service.catalog.to_dict()

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


def create_app(root=None,resource_root=None,*,mount_web=True):
    service=ComposableService(root or os.environ.get("MORLA_SYSTEM_STORE",str(Path.home()/".local/share/morla")),
                              resource_root=resource_root)
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
    dist=Path(__file__).parents[1]/"web/dist"
    if not dist.is_dir():
        try:
            from ament_index_python.packages import get_package_share_directory
            dist=Path(get_package_share_directory("morla_web_manager"))/"web/dist"
        except ImportError: pass
    if mount_web and dist.is_dir(): app.mount("/",StaticFiles(directory=dist,html=True),name="web")
    return app


def main():
    import uvicorn
    uvicorn.run(create_app(),host=os.environ.get("MORLA_WEB_MANAGER_HOST","127.0.0.1"),
                port=int(os.environ.get("MORLA_WEB_MANAGER_PORT","8080")))

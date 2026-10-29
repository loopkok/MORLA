"""Start a disarmed Quest/CR5 composable session and its local control UI."""
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
    from morla_web_manager.session_cli import main as run
    return run(configure_app=attach_control_page,default_store='~/.local/share/morla-cr5-vr')

import json, os
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse

ROOT = os.environ.get('DIST_ROOT', '/home/loopkok/MORLA/src/common/morla_web_manager/web/dist')

component = {
    'plugin_id': 'morla.demo.arm', 'plugin_version': '1.0.0', 'kind': 'actuator',
    'ports': {
        'position': {'direction': 'output', 'semantic': 'joint_position', 'dim': 2, 'unit': 'rad'},
        'position_target': {'direction': 'input', 'semantic': 'joint_position', 'dim': 2, 'unit': 'rad'},
    },
    'joints': [{'name': 'j1', 'unit': 'rad', 'lower': -3, 'upper': 3},
               {'name': 'j2', 'unit': 'rad', 'lower': -3, 'upper': 3}],
    'geometry': {'frames': ['base', 'mount']},
    'defaults': {},
    'configuration_schema': {'type': 'object', 'properties': {}},
    'web': {'label': '演示机械臂'},
    'backend_variants': {'sim': {'available': True}, 'fake': {'available': True}},
}

assembly = {
    'assembly_version': 1, 'assembly_id': 'demo_sim', 'revision': 1,
    'title': '演示仿真工作站',
    'metadata': {'simulation': {'mode': 'kinematic', 'display': 'web'}},
    'instances': {'arm': {'plugin': 'morla.demo.arm', 'version': '1.0.0', 'backend': 'sim'}},
    'mounts': [], 'bindings': [], 'control_groups': {},
    'capture': {'state': [], 'action': [], 'images': {}, 'action_source': 'command'},
}
resolved = {'assembly_id': 'demo_sim', 'revision': 1, 'assembly_hash': 'x', 'bindings': [],
            'feature_contract': {'state_dim': 2, 'action_dim': 2, 'cameras': {}}, 'diagnostics': []}


CURRENT = {'mode': os.environ.get('MODE', 'kinematic'), 'display': os.environ.get('DISPLAY', 'web'), 'status': 'ACTIVE'}


def session_state(display=None, mode=None):
    display = display or CURRENT['display']
    mode = mode or CURRENT['mode']
    return {'session_id': 'demo-session', 'status': CURRENT['status'], 'assembly_id': 'demo_sim',
            'revision': 1, 'recording': False,
            'simulation': {'mode': mode, 'display': display, 'viewer': {'status': 'closed'}},
            'operator': {'activity': 'paused', 'paused': True}, 'inputs': {},
            'controllers': {}, 'components': {}, 'policy': {'running': False},
            'diagnostic_log': {'state': 'not_started', 'written': {}, 'dropped': {}, 'queued': 0}}


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=ROOT, **k)

    def log_message(self, *a):
        pass

    def _json(self, value, status=200):
        body = json.dumps(value).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path.startswith('/api/v2'):
            if path == '/api/v2/components':
                return self._json({'components': [component]})
            if path == '/api/v2/assemblies':
                return self._json([{'assembly_id': 'demo_sim', 'revision': 1}])
            if path == '/api/v2/templates':
                return self._json([])
            if path == '/api/v2/active-session':
                return self._json(session_state() if os.environ.get('ACTIVE') else None)
            if path.startswith('/api/v2/assemblies/'):
                return self._json({'assembly': assembly, 'deployment': {}, 'resolved': resolved})
            if path.endswith('/state'):
                return self._json(session_state())
            return self._json({'detail': 'not found'}, 404)
        if path == '/':
            self.path = '/index.html'
        return super().do_GET()

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get('Content-Length') or 0)
        body = json.loads(self.rfile.read(length) or b'{}')
        if path == '/api/v2/deployments/preflight':
            return self._json({'compile_valid': True, 'preflight': {'ready': True, 'components': []}, 'resolved': resolved})
        if path.startswith('/api/v2/sessions/demo-session/operations'):
            print('POST operations', body, flush=True)
            if body.get('operation') == 'stop':
                CURRENT['status'] = 'STOPPED'
            return self._json({'status': 'SUCCEEDED', 'request_id': body.get('request_id')})
        if path == '/api/v2/sessions/demo-session/simulation':
            print('POST simulation', body, flush=True)
            CURRENT.update({k: v for k, v in body.items() if k in ('mode', 'display')})
            return self._json(session_state()['simulation'])
        return self._json({'detail': 'not found'}, 404)


ThreadingHTTPServer(('127.0.0.1', int(os.environ.get('PORT', '18082'))), Handler).serve_forever()

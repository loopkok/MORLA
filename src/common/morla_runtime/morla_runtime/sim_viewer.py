"""Local MuJoCo viewer of a runtime snapshot; never a second simulator/writer.

A separate process keeps GLFW/main-thread requirements and a closed window
independent from physics and the web server (mjpython is used on macOS).
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time


class LocalViewer:
    def __init__(self, session):
        self.session = session
        self.process = None
        self.stop_event = threading.Event()
        self.error = None

    def open(self):
        if self.process is not None and self.process.poll() is None:
            return
        if self.process is not None:
            self.close()
        executable = sys.executable
        if sys.platform == 'darwin':
            executable = shutil.which('mjpython') or str(Path(sys.executable).with_name('mjpython'))
            if not Path(executable).is_file():
                raise RuntimeError('LOCAL_VIEWER_REQUIRES_MJPYTHON_ON_MACOS')
        self.directory = tempfile.TemporaryDirectory(prefix='morla-viewer-')
        root = Path(self.directory.name)
        (root/'model.xml').write_text(self.session.artifact['xml'])
        self.log = (root/'viewer.log').open('w+')
        self.stop_event.clear()
        self.error = None
        self._publish(root)
        self.process = subprocess.Popen([executable, str(Path(__file__).resolve()), str(root)],
                                        stdout=self.log, stderr=self.log)
        self.thread = threading.Thread(target=self._pump, args=(root,), daemon=True, name='morla-viewer-snapshot')
        self.thread.start()

    def _publish(self, root):
        with self.session.state_lock:
            value = {'qpos': self.session.qpos_snapshot.tolist(), 'time': self.session.time_snapshot}
        temporary = root/'state.tmp'
        temporary.write_text(json.dumps(value))
        os.replace(temporary, root/'state.json')

    def _pump(self, root):
        try:
            while not self.stop_event.wait(1/30):
                if self.process.poll() is not None:
                    if self.process.returncode:
                        self.log.flush()
                        self.error = (root/'viewer.log').read_text()[-2000:]
                    return
                self._publish(root)
        except Exception as exc:
            self.error = str(exc)

    def status(self):
        return {'status': 'running' if self.process and self.process.poll() is None else 'closed',
                'error': self.error}

    def close(self):
        self.stop_event.set()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1)
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        if hasattr(self, 'log'):
            self.log.close()
            self.directory.cleanup()
        self.process = None


def main():
    import mujoco
    import mujoco.viewer
    root = Path(sys.argv[1])
    model = mujoco.MjModel.from_xml_path(str(root/'model.xml'))
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2
        viewer.cam.lookat[:] = [0, 0, .5]
        while viewer.is_running():
            sample = json.loads((root/'state.json').read_text())
            with viewer.lock():
                data.qpos[:] = sample['qpos']
                data.time = sample['time']
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1/60)


if __name__ == '__main__':
    main()

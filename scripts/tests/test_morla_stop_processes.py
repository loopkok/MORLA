"""Selection boundaries and cleanup against isolated dummy processes only."""
import contextlib
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
from morla_stop_processes import Process, ancestors, is_morla_process, select_targets


class SelectionTests(unittest.TestCase):
    def proc(self, pid, ppid, args, cwd='/home/robot/MORLA'):
        return Process(pid, ppid, '123', tuple(args), cwd=cwd)

    def test_workspace_installed_orphan_node(self):
        node = self.proc(100, 1, ['/usr/bin/python3', '/home/robot/MORLA/install/nero_robot_control/lib/nero_robot_control/nero_robot_driver'])
        self.assertTrue(is_morla_process(node, '/home/robot/MORLA'))

    def test_ros_launch_recognized_without_workspace_argument(self):
        node = self.proc(100, 1, ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'morla_bringup', 'robot.launch.py'])
        self.assertTrue(is_morla_process(node, '/home/robot/MORLA'))

    def test_other_web_ports_dev_server(self):
        node = self.proc(100, 1, ['node', '/home/robot/MORLA/src/common/morla_web_manager/web/node_modules/.bin/vite', '--port', '9090'])
        self.assertTrue(is_morla_process(node, '/home/robot/MORLA'))

    def test_unrelated_services_and_cwd_alone_are_not_enough(self):
        for args in [['sshd'], ['node','/srv/other-web/server.js'], ['python3','-m','vllm.entrypoints.openai.api_server'], ['ros2','launch','other_robot','start.launch.py']]:
            self.assertFalse(is_morla_process(self.proc(100, 1, args), '/home/robot/MORLA'))

    def test_selects_descendants_and_explicit_8080_owner_but_not_ssh(self):
        processes = {
            20: self.proc(20,1,['sshd']),
            21: self.proc(21,20,['bash','nero_web_start.sh']),
            22: self.proc(22,21,['python3','morla_stop_processes.py']),
            100: self.proc(100,1,['ros2','launch','morla_bringup','robot.launch.py']),
            101: self.proc(101,100,['robot_state_publisher']),
            102: self.proc(102,101,['generic-child']),
            200: self.proc(200,1,['other-web-server']),
            300: self.proc(300,1,['unrelated-worker']),
        }
        protected=ancestors(processes,{22})
        self.assertEqual(set(select_targets(processes,'/home/robot/MORLA',{200},protected)), {100,101,102,200})
        self.assertTrue({1,20,21,22}<=protected)

    def test_start_script_and_helper_are_protected_even_when_matching(self):
        processes={20:self.proc(20,1,['ros2','launch','morla_web_manager','web_manager.launch.py']),21:self.proc(21,20,['python3','helper.py'])}
        self.assertEqual(select_targets(processes,'/home/robot/MORLA',set(),ancestors(processes,{21})),{})


@unittest.skipUnless(sys.platform.startswith('linux'), 'requires Linux /proc and ss')
class DummyCleanupTests(unittest.TestCase):
    def test_dry_run_leaves_dummy_alive_then_cleanup_releases_its_port(self):
        with tempfile.TemporaryDirectory(prefix='morla-cleanup-test-') as temp:
            # Reserve an available test port, never touch 8080 or actual MORLA.
            with contextlib.closing(socket.socket()) as reserve:
                reserve.bind(('127.0.0.1',0))
                port=reserve.getsockname()[1]
            script=Path(temp)/'install'/'dummy_web.py'
            script.parent.mkdir()
            script.write_text('import http.server, sys\nhttp.server.HTTPServer(("127.0.0.1",int(sys.argv[1])),http.server.BaseHTTPRequestHandler).serve_forever()\n')
            process=subprocess.Popen([sys.executable,str(script),str(port)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            try:
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    try:
                        with socket.create_connection(('127.0.0.1',port),timeout=.1):break
                    except OSError:time.sleep(.05)
                else:self.fail('dummy server not ready')
                command=[sys.executable,str(SCRIPTS/'morla_stop_processes.py'),'--workspace',temp,'--web-port',str(port)]
                preview=subprocess.run(command+['--dry-run'],capture_output=True,text=True,timeout=5)
                self.assertEqual(preview.returncode,0,preview.stderr)
                self.assertIn(str(process.pid),preview.stdout)
                self.assertIsNone(process.poll())
                stopped=subprocess.run(command,capture_output=True,text=True,timeout=10)
                self.assertEqual(stopped.returncode,0,stopped.stdout+stopped.stderr)
                process.wait(timeout=2)
                with contextlib.closing(socket.socket()) as sock:
                    self.assertNotEqual(sock.connect_ex(('127.0.0.1',port)),0)
            finally:
                if process.poll() is None:process.kill()
                process.wait()


if __name__=='__main__':
    unittest.main()

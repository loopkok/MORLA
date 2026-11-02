"""MuJoCo real-time viewer mirroring the running CR5 session.

The composed session owns the authoritative simulation in-process (headless);
this tool loads the same versioned MJCF and drives a passive viewer from the
session's arm joint state, so what you see is the state the controller solved.
Read-only: it never commands the robot.

Usage: python3 tools/cr5_live_viewer.py [--url http://127.0.0.1:8080] [--hz 50]
"""
import argparse
import json
import sys
import time
import urllib.request

import mujoco
import mujoco.viewer
import numpy as np

MODEL = 'src/robot_description/mjcf/cr5/cr5.xml'
JOINTS = tuple(f'joint{i}' for i in range(1, 7))


def load_model(path):
    import os
    path = os.path.abspath(path)
    # from_xml_string resolves meshdir against the cwd; the MJCF uses a
    # relative meshdir, so anchor it at the model directory.
    os.chdir(os.path.dirname(path))
    xml = open(path, encoding='utf-8').read()
    # The composed scene adds a floor; mirror it so the viewer has a ground.
    floor = ('<geom name="floor" type="plane" size="3 3 0.01" '
             'rgba=".32 .34 .38 1" contype="0" conaffinity="0"/>')
    xml = xml.replace('<worldbody>', '<worldbody>\n    ' + floor, 1)
    return mujoco.MjModel.from_xml_string(xml)


def fetch(url):
    with urllib.request.urlopen(url + '/api/v2/active-session', timeout=1) as r:
        state = json.load(r)
    return state


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8080')
    parser.add_argument('--model', default=MODEL)
    parser.add_argument('--hz', type=float, default=50.0)
    args = parser.parse_args()

    model = load_model(args.model)
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINTS]
    vadr = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINTS]
    period = 1.0 / args.hz

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.1, -0.2, 0.45]
        viewer.cam.distance = 2.4
        viewer.cam.elevation = -18
        viewer.cam.azimuth = 135
        missing = 0
        status = ''
        while viewer.is_running():
            started = time.monotonic()
            try:
                state = fetch(args.url)
                status = state.get('status', '')
                arm = state.get('components', {}).get('arm', {})
                ports = arm.get('ports', {})
                q = ports.get('position', {}).get('values')
                v = ports.get('velocity', {}).get('values')
                if q and len(q) == 6:
                    data.qpos[qadr] = q
                    if v and len(v) == 6:
                        data.qvel[vadr] = v
                    mujoco.mj_forward(model, data)
                    missing = 0
                else:
                    missing += 1
            except Exception as exc:  # session stopped/restarting: keep the window
                missing += 1
                if missing == 1:
                    print(f'state unavailable: {exc}', file=sys.stderr)
            viewer.set_texts((None, None, None,
                              status if missing == 0 else 'no session'))
            viewer.sync()
            time.sleep(max(0.0, period - (time.monotonic() - started)))


if __name__ == '__main__':
    main()

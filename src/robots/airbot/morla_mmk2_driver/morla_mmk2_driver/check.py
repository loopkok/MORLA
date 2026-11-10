"""Explicit read-only endpoint check. Never calls activate or set_goal."""
import argparse
import json
import time
from pathlib import Path
from .session import DeviceSession
from .contract import PREFIX


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resolved',required=True,help='Compiled resolved JSON')
    parser.add_argument('--duration',type=float,default=2.)
    options=parser.parse_args(args)
    if not .1<=options.duration<=30:parser.error('duration must be .1..30 seconds')
    resolved=json.loads(Path(options.resolved).read_text())
    rows={r['instance_id']:r for r in resolved['instances'] if r['plugin_id'].startswith(PREFIX) and not r['plugin_id'].endswith('.camera')}
    if not rows:parser.error('no MMK2 SDK actuators')
    # Force read-only even if the deployment snapshot allows motion.
    for row in rows.values():row['parameters'].update(allow_motion=False,mapping_verified=False,hold_stop_accepted=False)
    session=DeviceSession(rows)
    try:
        session.start();time.sleep(options.duration)
        result={key:session.snapshot(key) for key in rows}
        print(json.dumps(result,ensure_ascii=False,indent=2))
        return 0 if all(s['ready'] for s in result.values()) else 1
    finally:session.close()

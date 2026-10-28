"""A non-activating runtime launcher for compiled snapshots."""
import argparse
import json
import signal
import sys
import time
from pathlib import Path
from .session import RuntimeSession


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('resolved', help='Trusted compiled resolved_system.json (hash required)')
    parser.add_argument('--once', action='store_true', help='Start disarmed, report, then stop')
    parser.add_argument('--preflight', action='store_true', help='Report installed backend metadata without connecting')
    args = parser.parse_args(argv)
    runtime = None
    started = False
    try:
        runtime = RuntimeSession(json.loads(Path(args.resolved).read_text()))
        if args.preflight:
            report = runtime.preflight()
            print(json.dumps(report, ensure_ascii=False), flush=True)
            return 0 if report['ready'] else 2
        runtime.start()
        started = True
        print(json.dumps(runtime.state(), ensure_ascii=False), flush=True)
        if not args.once:
            def interrupt(*_):
                raise KeyboardInterrupt
            signal.signal(signal.SIGTERM, interrupt)
            while True:
                runtime.tick()
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(json.dumps({'ok': False, 'error_code': str(exc).split(':', 1)[0], 'error': str(exc)},
                         ensure_ascii=False), file=sys.stderr, flush=True)
        return 2
    finally:
        if runtime is not None and started:
            runtime.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

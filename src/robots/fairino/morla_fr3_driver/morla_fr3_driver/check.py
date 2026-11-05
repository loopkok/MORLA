"""Read-only FR3 commissioning report. Never enables, changes mode or sends targets."""
import argparse
import json
import math
import time
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error('--seconds must be in (0, 60]')
    from morla_core.resource_locator import package_share
    parameters = json.loads((package_share('morla_fr3_driver') / 'descriptor.json').read_text())['defaults']
    parameters['host'] = args.host
    from .transport import FR3Transport
    from .kinematics import FR3Kinematics
    import numpy as np
    from scipy.spatial.transform import Rotation
    transport = None
    report = {'host': args.host, 'mode': 'read_only', 'motion_test': 'NOT_RUN'}
    try:
        transport = FR3Transport(parameters)
        samples = {}
        until = time.monotonic() + args.seconds
        while time.monotonic() < until:
            sample = transport.read()
            samples[sample['sequence']] = sample
            time.sleep(.02)
        sample = list(samples.values())[-1]
        q = sample['values']
        pose = transport.forward_kinematics(q)
        predicted = FR3Kinematics(parameters).fk(q)
        position_error = float(np.linalg.norm(predicted[:3, 3] - np.array(pose[:3]) / 1000))
        rotation_error = float(Rotation.from_matrix(
            predicted[:3, :3].T @ Rotation.from_euler('xyz', pose[3:], degrees=True).as_matrix()).magnitude())
        report.update(status='PASS' if len(samples) > 1 and position_error < .002 and rotation_error < .02 else 'FAIL',
                      distinct_samples=len(samples), last_feedback=sample, sdk_fk_mm_deg=pose,
                      dh_fk_m=predicted.tolist(), position_error_m=position_error, rotation_error_rad=rotation_error,
                      stamp_domain='host_monotonic_ns_on_complete_CNDE_frame')
    except Exception as exc:
        report.update(status='FAIL', error=f'{type(exc).__name__}: {exc}')
    finally:
        if transport is not None:
            try:
                transport.close()
            except Exception as exc:
                report.update(status='FAIL', close_error=str(exc))
    encoded = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
    print(encoded)
    if args.output:
        args.output.write_text(encoded + '\n')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

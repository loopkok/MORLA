"""Print JSON reports: python3 -m vader5_sdk [--device /dev/hidrawN]."""
import argparse
import json
import sys
from . import Vader5, BUTTON_NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="Explicit Linux hidraw device (optional)")
    args = parser.parse_args()
    try:
        with Vader5(args.device) as pad:
            while True:
                for s in pad.read_batch():
                    print(json.dumps(dict(
                        received_at=s.received_at,
                        buttons={name: s.pressed(name) for name in BUTTON_NAMES},
                        left_stick=s.left_stick, right_stick=s.right_stick,
                        triggers=s.triggers, gyro_rad_s=s.gyro_rad_s,
                        accel_m_s2=s.accel_m_s2, unknown_bits=s.raw.unknown_bits,
                        firmware=pad.info.firmware if pad.info else None,
                    )), flush=True)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, TimeoutError) as exc:
        print(f"Vader5: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

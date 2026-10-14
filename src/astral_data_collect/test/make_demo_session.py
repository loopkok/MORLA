#!/usr/bin/env python3
"""生成一个合成 demo session（3 段健康 episode），用于手工/CLI 端到端验证。

用法：
  /usr/bin/python3 make_demo_session.py /tmp/astral_demo/pick_place
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from conftest import write_raw_episode  # noqa: E402


def main() -> None:
    root = sys.argv[1] if len(sys.argv) > 1 else "/tmp/astral_demo/pick_place"
    tasks = ["pick up the cube", "pick up the cube", "put down the cube"]
    for i, task in enumerate(tasks):
        write_raw_episode(
            os.path.join(root, f"episode{i:06d}"), duration_s=2.0, task=task
        )
    print(f"demo session -> {root} ({len(tasks)} episodes)")


if __name__ == "__main__":
    main()

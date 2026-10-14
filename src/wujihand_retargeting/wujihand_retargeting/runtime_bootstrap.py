"""Preload libstdc++ / nlopt for wuji_sdk retarget (same as rob_station)."""

from __future__ import annotations

import ctypes
import os
import sys

_BOOTSTRAPPED = False


def bootstrap_retarget_runtime() -> None:
    global _BOOTSTRAPPED
    if _BOOTSTRAPPED:
        return
    _BOOTSTRAPPED = True

    prefix = os.environ.get(
        "WUJI_NLOPT_PREFIX",
        os.path.expanduser("~/.local/py310-extra"),
    )
    lib_dir = os.path.join(prefix, "lib")
    for name in ("libstdc++.so.6", "libgcc_s.so.1"):
        path = os.path.join(lib_dir, name)
        if os.path.isfile(path):
            ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)

    if sys.version_info[:2] == (3, 10):
        site_packages = os.path.join(prefix, "lib", "python3.10", "site-packages")
        if os.path.isdir(site_packages) and site_packages not in sys.path:
            sys.path.insert(0, site_packages)

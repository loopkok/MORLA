"""Host video-device auto-scan (no hardcoded device nodes).

Enumerates ``/dev/video*`` and keeps one **capture-capable** node per physical
device:

  * nodes without ``V4L2_CAP_VIDEO_CAPTURE`` (e.g. metadata nodes that USB
    cameras expose alongside the video node) are skipped;
  * nodes belonging to the same physical device (sysfs parent) are de-duped.

When a device exposes several capture nodes (RealSense D435i: depth / IR /
color), pick the **color-like** format (YUYV / MJPG / RGB) rather than the
lowest-numbered node. IR GREY and Z16 depth are never chosen as the stream.
Color is often ``VIDEO_CAPTURE_MPLANE`` (``/dev/video8``); IR is classic
capture — both buffer types are enumerated.

Label convention: ``video{N}`` (the device node basename).
"""

from __future__ import annotations

import fcntl
import glob
import os
import struct
import subprocess
from typing import Any

# VIDIOC_QUERYCAP = _IOR('V', 0, struct v4l2_capability[104])
_VIDIOC_QUERYCAP = 0x80685600
_V4L2_CAP_VIDEO_CAPTURE = 0x00000001
_V4L2_CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
_V4L2_CAP_DEVICE_CAPS = 0x80000000

_COLOR_FOURCC = frozenset({
    "YUYV", "YUY2", "UYVY", "MJPG", "JPEG", "RGB3", "BGR3", "RGBP",
    "NV12", "NV21", "YU12", "I420",
})
# 压缩彩色格式单独成档：MJPG/JPEG 与 YUYV 同为彩色（100 分），但 UVC 恒把
# 未压缩格式枚举在前，同分稳定排序会让 YUYV 胜出 —— D435i 1080p30 YUYV 需
# ~995Mbps，直接打满共享 USB2 总线（实测采集只剩 3.3fps）。MJPG ~40Mbps，
# 同分辨率帧率下是唯一可行项；cv2 端解码开销可忽略。
_MJPEG_FOURCC = frozenset({"MJPG", "JPEG"})
_IR_FOURCC = frozenset({
    "GREY", "GRAY", "Y8", "Y8I", "Y10", "Y10I", "Y12", "Y12I", "Y16", "Y16I",
    "W10", "MONO",
})
_DEPTH_FOURCC = frozenset({"Z16", "Z16H", "INVZ", "INZI"})

# VIDIOC_ENUM_FMT = _IOWR('V', 2, struct v4l2_fmtdesc[64])
_VIDIOC_ENUM_FMT = 0xC0405602
_V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
_V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9

# Color-like score floor. IR GREY=10, depth=0, unknown=1. Never stream IR.
_MIN_STREAM_SCORE = 50


def _is_capture_node(path: str) -> bool:
    """True when the v4l2 node supports video capture."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        buf = bytearray(104)
        fcntl.ioctl(fd, _VIDIOC_QUERYCAP, buf, True)
        (capabilities,) = struct.unpack_from("<I", buf, 84)
        (device_caps,) = struct.unpack_from("<I", buf, 88)
        caps = device_caps if (capabilities & _V4L2_CAP_DEVICE_CAPS) else capabilities
        return bool(caps & (_V4L2_CAP_VIDEO_CAPTURE | _V4L2_CAP_VIDEO_CAPTURE_MPLANE))
    except OSError:
        return False
    finally:
        os.close(fd)


def _physical_group(node: str) -> str:
    """Group key = the USB device (not interface) owning this video node."""
    real = os.path.realpath(f"/sys/class/video4linux/{node}/device")
    suffix = f"/video4linux/{node}"
    if real.endswith(suffix):
        real = real[: -len(suffix)]
    # D435i RGB is often a sibling USB interface (1-2:1.3) of depth/IR (1-2:1.0).
    # Group by the USB device so color can beat IR on the same camera.
    base = os.path.basename(real)
    if ":" in base:
        parent = os.path.dirname(real)
        if parent and parent != "/":
            return parent
    return real


def sysfs_name(node: str) -> str:
    """Human-readable camera name ('' when unknown)."""
    try:
        with open(f"/sys/class/video4linux/{node}/name", "r", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def _node_index(node: str) -> int:
    try:
        return int(node[len("video"):])
    except ValueError:
        return 0


def _fourcc_str(pixelformat: int) -> str:
    return "".join(chr((pixelformat >> (8 * i)) & 0xFF) for i in range(4))


def _enum_fourccs(path: str) -> list[str]:
    """Formats advertised by VIDIOC_ENUM_FMT (no v4l2-ctl needed).

    D435i color is often ``VIDEO_CAPTURE_MPLANE``; IR/depth use classic
    ``VIDEO_CAPTURE``. Enumerate both or the RGB node looks empty and IR wins.
    """
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return []
    out: list[str] = []
    seen: set[str] = set()
    try:
        for buf_type in (
            _V4L2_BUF_TYPE_VIDEO_CAPTURE,
            _V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        ):
            for index in range(32):
                buf = bytearray(64)
                struct.pack_into("<III", buf, 0, index, buf_type, 0)
                try:
                    fcntl.ioctl(fd, _VIDIOC_ENUM_FMT, buf, True)
                except OSError:
                    break
                (pixelformat,) = struct.unpack_from("<I", buf, 44)
                fourcc = _fourcc_str(pixelformat).strip()
                if fourcc and fourcc not in seen:
                    seen.add(fourcc)
                    out.append(fourcc)
    finally:
        os.close(fd)
    return out


def _current_fourcc(path: str) -> str:
    """Current Pixel Format via v4l2-ctl (optional; ioctl enum is primary)."""
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", path, "--get-fmt-video"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=1.5,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    for line in out.splitlines():
        if "Pixel Format" not in line:
            continue
        if "'" in line:
            return line.split("'")[1].strip()
        return ""
    return ""


def _fourcc_score(fourcc: str) -> int:
    """Higher = better for RGB webcam streaming."""
    key = fourcc.strip()
    if key in _MJPEG_FOURCC or fourcc in _MJPEG_FOURCC:
        return 110
    if key in _COLOR_FOURCC or fourcc in _COLOR_FOURCC:
        return 100
    if key in _IR_FOURCC or fourcc in _IR_FOURCC:
        return 10
    if key in _DEPTH_FOURCC or fourcc in _DEPTH_FOURCC:
        return 0
    return 1


def _name_score(name: str) -> int:
    """sysfs name hint when fourcc is unavailable."""
    n = name.lower()
    if any(k in n for k in ("rgb", "color", "colour")):
        return 50
    if "infrared" in n or n.endswith(" ir") or " ir " in n:
        return 10
    if "depth" in n:
        return 0
    return 1


def _best_score(fourccs: list[str], current: str, sysfs: str) -> tuple[int, str]:
    """Return (score, representative fourcc) for ranking this node.

    D435i IR stereo advertises UYVY next to GREY/Y8I/Y12I. That UYVY is packed
    infrared, not RGB. Any IR fourcc classifies the node as IR (score 10) so it
    cannot tie with the color node and win on lower /dev/video index.
    """
    seen: list[str] = []
    for f in list(fourccs) + ([current] if current else []):
        key = f.strip()
        if key and key not in seen:
            seen.append(key)
    ir = [f for f in seen if f in _IR_FOURCC]
    if ir:
        return 10, ir[0]
    depth = [f for f in seen if f in _DEPTH_FOURCC]
    if depth:
        return 0, depth[0]
    scored = [(_fourcc_score(f), f) for f in seen]
    if scored:
        scored.sort(key=lambda x: -x[0])
        return scored[0]
    return _name_score(sysfs), current or ""


def enumerate_capture_devices() -> list[dict[str, Any]]:
    """One entry per physical capture device, ordered by device node number."""
    by_group: dict[str, list[dict[str, Any]]] = {}
    for path in sorted(
        glob.glob("/dev/video*"), key=lambda p: int(p[len("/dev/video"):])
    ):
        node = os.path.basename(path)
        if not _is_capture_node(path):
            continue
        fourccs = _enum_fourccs(path)
        current = _current_fourcc(path)
        name = sysfs_name(node)
        score, fourcc = _best_score(fourccs, current, name)
        rec = {
            "label": node,
            "device": path,
            "sysfs_name": name,
            "fourcc": fourcc,
            "score": score,
        }
        by_group.setdefault(_physical_group(node), []).append(rec)

    out: list[dict[str, Any]] = []
    for recs in by_group.values():
        recs.sort(key=lambda r: (-int(r["score"]), _node_index(str(r["label"]))))
        best = recs[0]
        if int(best["score"]) < _MIN_STREAM_SCORE:
            # Depth (0) / IR GREY (10) / unknown. Do not stream IR as "the camera".
            continue
        fourcc = str(best.get("fourcc", ""))
        out.append({
            "label": best["label"],
            "device": best["device"],
            "sysfs_name": best["sysfs_name"],
            "fourcc": fourcc,
            "score": int(best["score"]),
            "force_mjpg": fourcc.strip() in ("MJPG", "JPEG"),
        })
    out.sort(key=lambda d: _node_index(str(d["label"])))
    return out


def stable_fingerprints(node: str) -> list[str]:
    """Identity fingerprints for a video node, most stable first.

    by-id   : ties to the physical camera (survives re-plug / port change)
    by-path : ties to the physical USB port (survives device swap)
    sysfs   : device model name, last-resort hint (not unique per camera)

    Users match these against `label_aliases` patterns (substring match) to
    pin a stable stream label regardless of the kernel's videoN numbering.
    """
    fps: list[str] = []
    for link in ("/dev/v4l/by-id", "/dev/v4l/by-path"):
        try:
            for entry in os.scandir(link):
                if os.path.realpath(entry.path) == os.path.realpath(f"/dev/{node}"):
                    fps.append(entry.name)
        except OSError:
            pass
    fps.append(sysfs_name(node))
    return [f for f in fps if f]


def apply_label_aliases(
    devices: list[dict[str, Any]], aliases: list[str]
) -> tuple[list[dict[str, Any]], list[str]]:
    """Rewrite scanned labels via ``<pattern>=<label>`` alias rules.

    A device matches a rule when any of its stable fingerprints (by-id /
    by-path / sysfs name) contains the pattern as a substring. Devices may
    carry a pre-computed ``fingerprints`` list (used by tests and by callers
    that already resolved links); otherwise it is derived from the node via
    :func:`stable_fingerprints`. First matching rule wins per device.

    Rule validation (against the kernel labels of the current scan):
      * target equals the matched device's own label -> allowed no-op (pinning
        the current name); the device is consumed either way;
      * target equals the kernel label of a *different* device -> rejected
        (renaming would create a duplicate label);
      * two devices matched by one rule -> only the first is renamed.
    A rule matching no device at all is reported too (usually a typo — or a
    camera that is unplugged, which is exactly what you want to hear about).

    Returns (devices, warnings).
    """
    rules: list[tuple[str, str]] = []
    warnings: list[str] = []
    for raw in aliases:
        entry = str(raw).strip()
        if not entry or "=" not in entry:
            warnings.append(f"alias ignored (want 'pattern=label'): {entry!r}")
            continue
        pattern, _, label = entry.partition("=")
        pattern, label = pattern.strip(), label.strip()
        if not pattern or not label:
            warnings.append(f"alias ignored (empty side): {entry!r}")
            continue
        rules.append((pattern, label))

    def _fps(dev: dict[str, Any]) -> list[str]:
        return [str(f) for f in dev.get("fingerprints") or ()] or \
            stable_fingerprints(str(dev["label"]))

    current_labels = {str(d["label"]) for d in devices}
    renamed: set[int] = set()   # device ids already assigned a stable label
    used: set[str] = set()      # stable labels assigned so far
    for pattern, label in rules:
        hits = [d for d in devices
                if id(d) not in renamed and any(pattern in f for f in _fps(d))]
        if not hits:
            warnings.append(
                f"alias '{pattern}={label}' matched no scanned device "
                "(typo, or the camera is not plugged in)"
            )
            continue
        if len(hits) > 1:
            warnings.append(
                f"alias '{pattern}={label}' matches {len(hits)} devices "
                f"({', '.join(str(d['label']) for d in hits)}): "
                "only the first will be renamed"
            )
        target = hits[0]
        if label in current_labels and str(target["label"]) != label:
            warnings.append(
                f"alias '{pattern}={label}' rejected: '{label}' is the kernel "
                "label of another scanned device"
            )
            continue
        if label in used:
            warnings.append(
                f"alias '{pattern}={label}' rejected: '{label}' already "
                "assigned by an earlier alias in this list"
            )
            continue
        if str(target["label"]) != label:
            warnings.append(
                f"alias '{pattern}={label}' matched {target['label']} via "
                + next(f for f in _fps(target) if pattern in f)
            )
            target["label"] = label
        renamed.add(id(target))
        used.add(label)
    return devices, warnings


def _print_fingerprints() -> None:
    """CLI helper: print stable fingerprints of every capture device.

    Usage (on the robot):
      python3 -m quest3_video_streamer.scan
    Copy the substring of any fingerprint into a label_aliases rule.
    """
    devices = enumerate_capture_devices()
    if not devices:
        print("no capture-capable device found")
        return
    for dev in devices:
        node = str(dev["label"])
        print(f"{node}  ({dev['device']}, {dev['fourcc'] or 'fourcc?'}):")
        for f in stable_fingerprints(node):
            print(f"    {f}")


if __name__ == "__main__":
    _print_fingerprints()

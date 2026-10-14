"""MJPG 优先回归：采集低帧率根因修复。

根因：D435i 彩色节点同时枚举 YUYV 与 MJPG，且 UVC 驱动恒把 YUYV 列在前。
旧评分给所有彩色格式同分（100），稳定排序保留枚举序 → YUYV 胜出 →
force_mjpg=False → 1920x1080@30 未压缩流 ≈995Mbps 打满共享 USB2 总线，
实测采集只剩 3.3fps 并拖垮同总线两个 720p 腕部相机（7-10fps 抖动）。
MJPG 是压缩格式（~40Mbps），同分辨率帧率下是唯一可行选择。
"""
from __future__ import annotations

from quest3_video_streamer.scan import _best_score


def test_mjpg_wins_over_yuyv_regardless_of_enum_order():
    """UVC 枚举序（YUYV 在前）不应让未压缩格式胜出。"""
    # D435i 彩色节点的真实枚举序：YUYV index0，MJPG index1
    score, fourcc = _best_score(["YUYV", "MJPG"], "", "USB Video")
    assert fourcc == "MJPG", f"MJPG 应优先于 YUYV，得到 {fourcc}"
    assert score > 100

    # 逆序亦然（防御：不依赖枚举顺序）
    score, fourcc = _best_score(["MJPG", "YUYV"], "", "USB Video")
    assert fourcc == "MJPG"


def test_ir_trap_classification_preserved():
    """MJPG 提分不得破坏 IR 陷阱防护：节点内含任何 IR/depth 格式即整节点归类。

    （D435i 红外节点会把打包红外 UYVY 与 GREY 一起播——若按单格式取最高分，
    红外节点会凭 UYVY 被误选为彩色源。彩色>IR 的比较发生在物理节点之间，
    由 enumerate_capture_devices 按分数排序保证。）
    """
    score, fourcc = _best_score(["GREY", "UYVY"], "", "Infrared")
    assert score == 10
    score, fourcc = _best_score(["Z16", "YUYV"], "", "Depth")
    assert score == 0
    # 纯彩色节点 110/100 分，组间排序恒胜 IR(10)/depth(0) 节点
    assert _best_score(["MJPG"], "", "USB Video")[0] == 110
    assert _best_score(["YUYV"], "", "USB Video")[0] == 100


def test_yuyv_chosen_when_no_mjpg_advertised():
    """设备不播 MJPG 时退回 YUYV（保持现状行为，由 preset/降分辨率兜底）。"""
    score, fourcc = _best_score(["YUYV"], "", "USB Video")
    assert fourcc == "YUYV"
    assert score == 100

"""label_aliases 单测：扫描结果的稳定 label 改写规则。

不依赖真实 /dev 设备 —— stable_fingerprints 打桩，只测 apply_label_aliases
的规则语义（命中/未命中/多设备/冲突/畸形规则）与 streamer_node 集成点。
"""

import sys
import unittest
from unittest import mock

sys.path.insert(0, "src")

from quest3_video_streamer import scan as scan_mod
from quest3_video_streamer import streamer_node as sn


class DeviceToIndexTests(unittest.TestCase):
    """_device_to_index：auto_scan 时块 device 可能是 by-path/by-id 路径，
    必须还原成 video 索引（2026-09-20 崩：by-path 进 int() ValueError）。"""

    def test_video_devnode(self):
        self.assertEqual(sn._device_to_index("/dev/video6"), 6)
        self.assertEqual(sn._device_to_index("/dev/video0"), 0)

    def test_plain_int(self):
        self.assertEqual(sn._device_to_index(6), 6)
        self.assertEqual(sn._device_to_index("6"), 6)

    def test_by_path_suffix(self):
        # 无 /dev/v4l symlink 的主机：退化用 -video-indexN 后缀
        self.assertEqual(
            sn._device_to_index(
                "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.3-video-index0"
            ),
            0,
        )

    def test_by_id_suffix(self):
        self.assertEqual(
            sn._device_to_index(
                "/dev/v4l/by-id/usb-Intel_R_RealSense_TM_Depth_Camera_435i_254843065994-video-index0"
            ),
            0,
        )

    def test_resolves_symlink_when_present(self):
        # by-path symlink 存在时走 realpath → /dev/videoN
        with mock.patch("quest3_video_streamer.streamer_node.os.path.realpath",
                        return_value="/dev/video6"):
            self.assertEqual(
                sn._device_to_index(
                    "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.3-video-index0"
                ),
                6,
            )

    def test_garbage_raises(self):
        with self.assertRaises(ValueError):
            sn._device_to_index("not-a-device")


def _dev(label, fingerprints, fourcc="YUYV", score=100):
    return {
        "label": label, "device": f"/dev/{label}",
        "sysfs_name": fingerprints[-1] if fingerprints else "",
        "fourcc": fourcc, "score": score, "force_mjpg": False,
        "fingerprints": list(fingerprints),
    }


class StableFingerprintsTests(unittest.TestCase):
    def test_order_and_fallbacks(self):
        # by-id 优先，by-path 其次，sysfs 兜底；全无链接时只剩 sysfs。
        by_id = "/dev/v4l/by-id/usb-Intel_RealSense_1-video-index2"
        by_path = "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.3-video"

        class _Entry:
            def __init__(self, path):
                self.path = path
                self.name = path.rsplit("/", 1)[-1]

        def fake_scandir(path):
            return {
                "/dev/v4l/by-id": [_Entry(by_id)],
                "/dev/v4l/by-path": [_Entry(by_path)],
            }[path]

        with mock.patch.object(scan_mod.os, "scandir", side_effect=fake_scandir), \
             mock.patch.object(scan_mod.os.path, "realpath",
                               side_effect=lambda p: "video8" if p in
                               (by_id, by_path, "/dev/video8") else p), \
             mock.patch.object(scan_mod, "sysfs_name", return_value="Intel RealSense 435I"):
            fps = scan_mod.stable_fingerprints("video8")
        self.assertEqual(len(fps), 3)
        # 顺序：by-id 链接名 → by-path 链接名 → sysfs 名
        self.assertEqual(fps[0], "usb-Intel_RealSense_1-video-index2")
        self.assertEqual(fps[1], "pci-0000:00:14.0-usb-0:2:1.3-video")
        self.assertEqual(fps[2], "Intel RealSense 435I")

    def test_no_links_sysfs_only(self):
        with mock.patch.object(scan_mod.os, "scandir",
                               side_effect=OSError("no by-id dir")), \
             mock.patch.object(scan_mod, "sysfs_name",
                               return_value="USB Camera"):
            fps = scan_mod.stable_fingerprints("video0")
        self.assertEqual(fps, ["USB Camera"])


class ApplyLabelAliasesTests(unittest.TestCase):
    def _run(self, devices, aliases):
        return scan_mod.apply_label_aliases(devices, aliases)

    def test_hit_by_id(self):
        devs = [_dev("video8", ["usb-Intel_R._RealSense_435I_with_MIPI_Disassembler-video-index2",
                                "Intel RealSense 435I"])]
        out, warns = self._run(devs, ["Intel_R._RealSense=camera_head"])
        self.assertEqual(out[0]["label"], "camera_head")
        self.assertTrue(any("via" in w and "by-id" not in w or "via" in w
                            for w in warns))
        self.assertTrue(any("video8" in w for w in warns))  # 警告含旧名

    def test_no_match_keeps_kernel_label(self):
        devs = [_dev("video3", ["usb-SomeOtherCam-video-index0", "Other Cam"])]
        out, _ = self._run(devs, ["RealSense=video8"])
        self.assertEqual(out[0]["label"], "video3")

    def test_by_id_beats_sysfs_same_rule_set(self):
        # 规则按顺序首中即停：by-id 指纹在 sysfs 之前，先列的具体规则优先。
        devs = [_dev("video0", ["usb-CamAAA-video-index0", "CamAAA"])]
        out, _ = self._run(devs, ["CamAAA=wrist_left"])
        self.assertEqual(out[0]["label"], "wrist_left")

    def test_first_rule_wins_on_multi_match(self):
        devs = [_dev("video8", ["usb-Intel_R._RealSense_435I-video-index2",
                                "Intel RealSense 435I"])]
        out, _ = self._run(devs, ["RealSense=video8", "435I=camera_chest"])
        self.assertEqual(out[0]["label"], "video8")  # 第一条命中，第二条不再看

    def test_two_devices_two_labels(self):
        devs = [
            _dev("video8", ["usb-Intel_R._RealSense_435I-video-index2", "RealSense 435I"]),
            _dev("video0", ["usb-1080P_Camera-video-index0", "1080P Camera"]),
        ]
        out, _ = self._run(devs, ["RealSense=video8", "1080P=wrist_left"])
        self.assertEqual([d["label"] for d in out], ["video8", "wrist_left"])

    def test_collision_with_kernel_label_rejected(self):
        # 别名目标撞上扫描里另一台设备的 videoN label → 该规则拒用。
        devs = [
            _dev("video8", ["usb-Intel_R._RealSense-video-index2"]),
            _dev("video0", ["usb-1080P_Camera-video-index0"]),
        ]
        out, warns = self._run(devs, ["1080P=video8"])
        self.assertEqual(out[0]["label"], "video8")  # RealSense 未被改
        self.assertEqual(out[1]["label"], "video0")  # 规则被拒
        self.assertTrue(any("kernel label" in w for w in warns))

    def test_pinning_current_kernel_name_is_noop(self):
        # 最常见写法：把设备当前恰好叫 video8 的名字固化下来。合法 no-op，
        # 不允许被当成"与内核 label 冲突"而拒绝。
        devs = [_dev("video8", ["usb-Intel_R._RealSense_435I-video-index2"])]
        out, warns = self._run(devs, ["RealSense=video8", "435I=camera_chest"])
        self.assertEqual(out[0]["label"], "video8")
        self.assertFalse(any("rejected" in w for w in warns))

    def test_second_device_same_label_keeps_first(self):
        devs = [
            _dev("video0", ["usb-CamAAA-video-index0"]),
            _dev("video2", ["usb-CamAAA-clone-video-index0"]),  # 同款第二台
        ]
        out, warns = self._run(devs, ["CamAAA=video8"])
        self.assertEqual(out[0]["label"], "video8")
        self.assertEqual(out[1]["label"], "video2")
        self.assertTrue(any("matches 2 devices" in w for w in warns))

    def test_malformed_rules_ignored_with_warning(self):
        devs = [_dev("video0", ["usb-CamAAA-video-index0"])]
        for bad in ("noequals", "=empty_pattern", "empty_label=", "  ", ""):
            out, warns = self._run(devs, [bad])
            self.assertEqual(out[0]["label"], "video0", bad)
            self.assertTrue(warns, bad)

    def test_non_string_entries_coerced(self):
        devs = [_dev("video0", ["usb-CamAAA-video-index0"])]
        out, _ = self._run(devs, [123456])  # 非 str 条目不崩，走畸形忽略
        self.assertEqual(out[0]["label"], "video0")


class StreamerNodeIntegrationTests(unittest.TestCase):
    """_build_sources 的 auto_scan 分支确实把别名应用到了 infos/覆盖块键上。

    streamer_node import rclpy —— 无 ROS 环境时跳过（与 test_quest_layout
    同样的约束）。
    """

    def test_auto_scan_applies_aliases_before_spec_build(self):
        try:
            import quest3_video_streamer.streamer_node as sn
        except ImportError:  # pragma: no cover - 无 rclpy 的开发机
            self.skipTest("rclpy 不可用")
        from quest3_video_streamer import scan as scan_module

        captured = {}

        def fake_scan_spec(node, dev, found):
            captured["labels"] = [d["label"] for d in found]
            captured["dev"] = dev
            return {"type": "webcam", "webcam_index": 0, "preset": "720p30",
                    "fov_h_deg": 60.0, "label": dev["label"],
                    "force_mjpg": True, "layout": {"position": [0, 0, 1.8],
                                                   "distance": 1.8,
                                                   "size_multiplier": 0.28}}, {}

        scanned = [_dev("video9", ["usb-Intel_R._RealSense_435I-video-index2",
                                   "RealSense 435I"])]
        with mock.patch.object(scan_module, "enumerate_capture_devices",
                               return_value=scanned), \
             mock.patch.object(scan_module, "apply_label_aliases",
                               wraps=scan_module.apply_label_aliases) as spy, \
             mock.patch.object(sn, "_scan_spec", side_effect=fake_scan_spec), \
             mock.patch.object(sn, "_build_one_source", lambda node, spec: mock.Mock()):
            sn._build_sources(
                mock.Mock(),
                {"auto_scan": True,
                 "label_aliases": ["Intel_R._RealSense=video8"]},
            )
        self.assertTrue(spy.called)
        # 别名生效后，_scan_spec 看到的 label 已是稳定名
        self.assertEqual(captured["dev"]["label"], "video8")
        self.assertEqual(captured["labels"], ["video8"])


if __name__ == "__main__":
    unittest.main()

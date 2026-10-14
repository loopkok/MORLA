"""推理控制纯逻辑（无 ROS）：launch 参数构建 + cmd 白名单。"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from astral_web_monitor.config import (  # noqa: E402
    PI_COMMANDS,
    PI_LOG_ROOT,
    policy_launch_args,
    valid_infer_cmd,
)


def test_policy_launch_args_defaults() -> None:
    args = policy_launch_args({})
    assert args["backend_type"] == "remote"
    assert args["model"] == "pi05"
    assert args["host"] == "127.0.0.1"
    assert args["port"] == "8001"
    assert args["camera_image_size"] == "224"
    assert args["engine_mode"] == "queue_async"
    assert args["keyboard"] == "false"  # web 按钮取代键盘节点
    assert "metrics_log_file" not in args  # 日志默认关
    assert "log_dir" not in args


def test_policy_launch_args_overrides() -> None:
    args = policy_launch_args({
        "backend_type": "stub",
        "model": "pi05",
        "host": "192.168.1.50",
        "port": 9000,
        "camera_image_size": 224,
        "engine_mode": "rtc",
        "log": True,
    })
    assert args["host"] == "192.168.1.50"
    assert args["port"] == "9000"
    assert args["camera_image_size"] == "224"
    assert args["engine_mode"] == "rtc"
    assert args["model"] == "pi05"
    assert args["backend_type"] == "stub"
    # 日志开关 → 注入 log_dir 根目录（launch 自动建目录并落四类诊断流）
    assert args["log_dir"] == PI_LOG_ROOT
    assert "metrics_log_file" not in args


def test_policy_launch_args_int_coercion() -> None:
    # 前端可能传字符串数字，也要正确 int 化
    args = policy_launch_args({"port": "8002", "camera_image_size": "640"})
    assert args["port"] == "8002"
    assert args["camera_image_size"] == "640"


def test_policy_launch_args_shell_injection_sanitized() -> None:
    """对抗性：host 等经 bash -c 拼命令，shell 元字符必须被清洗（命令注入防护）。"""
    args = policy_launch_args({
        "host": "192.168.1.50; rm -rf /tmp/x",
        "backend_type": "remote; touch /tmp/pwned",
        "model": "act$(id)",
        "engine_mode": "queue_async`id`",
    })
    assert args["host"] == "192.168.1.50rm-rftmpx"  # 非法字符全剔除（分号/空格/斜杠），横杠是合法主机名字符
    assert args["backend_type"] == "remotetouchtmppwned"
    assert args["model"] == "actid"
    assert args["engine_mode"] == "queue_asyncid"
    # 没有可执行的 shell 元字符
    for v in args.values():
        assert not any(c in v for c in ";$(){}`|<>& \t\n")
    assert args["keyboard"] == "false"


def test_valid_infer_cmd_all_verbs() -> None:
    for verb in PI_COMMANDS:
        assert valid_infer_cmd(verb), f"{verb} 应合法"


def test_valid_infer_cmd_playback_with_source() -> None:
    assert valid_infer_cmd("playback:/data/ep0")
    assert valid_infer_cmd("playback:/data/ep0:3")


def test_valid_infer_cmd_rejects_junk() -> None:
    for bad in ("", " rm -rf /", "start", "policy;x", "stop --force", "s"):
        assert not valid_infer_cmd(bad), f"{bad!r} 应被拒"

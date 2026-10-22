"""Web owns optional input lifetimes; the selected Profile owns the router."""
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch")
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, RegisterEventHandler


@pytest.fixture
def module():
    path = Path(__file__).resolve().parents[1] / "launch" / "web_manager.launch.py"
    spec = importlib.util.spec_from_file_location("operator_web_launch", path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def context(**overrides):
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(voice_source="off", microphone_device="",
        pedal_device="", operator_config="", operator_dry_run="false", **overrides))
    return ctx


def test_plain_web_keeps_inputs_off_without_resolving_operator_package(module, monkeypatch):
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: pytest.fail(name))
    assert module._operator_inputs(context(), object()) == []


@pytest.mark.parametrize("source,pedal", [("quest", ""), ("host", ""), ("both", ""),
    ("off", "/dev/input/by-id/usb-pedal-event-kbd")])
def test_selected_inputs_never_start_second_router_and_share_web_lifecycle(module, monkeypatch, source, pedal):
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: "/tmp/operator")
    # A real launch Node is required by the target-action event matcher.
    from launch_ros.actions import Node
    web = Node(package="morla_web_manager", executable="morla_web_manager")
    ctx = context()
    ctx.launch_configurations.update(voice_source=source, pedal_device=pedal,
        microphone_device="alsa:device with spaces", operator_config="/tmp/custom config.yaml",
        operator_dry_run="true")
    actions = module._operator_inputs(ctx, web)
    assert isinstance(actions[0], RegisterEventHandler)
    assert isinstance(actions[1], IncludeLaunchDescription)
    arguments = dict(actions[1].launch_arguments)
    assert arguments == dict(start_router="false", voice_source=source, pedal_device=pedal,
        microphone_device="alsa:device with spaces", config="/tmp/custom config.yaml", dry_run="true")


def test_invalid_voice_source_is_rejected(module):
    ctx = context()
    ctx.launch_configurations["voice_source"] = "invalid"
    with pytest.raises(ValueError):
        module._operator_inputs(ctx, object())

"""Safe-by-default web monitor network binding."""
import os
import importlib.util
from pathlib import Path
import subprocess
import sys

from launch.actions import DeclareLaunchArgument

PACKAGE = Path(__file__).resolve().parents[1]


def test_web_host_defaults_to_loopback_and_can_be_overridden():
    env = {**os.environ, 'PYTHONPATH': str(PACKAGE)}
    env.pop('ASTRAL_WEB_MONITOR_HOST', None)
    result = subprocess.run(
        [sys.executable, '-c', 'from astral_web_monitor.config import WEB_HOST; print(WEB_HOST)'],
        env=env, capture_output=True, text=True, check=True,
    )
    assert result.stdout.strip() == '127.0.0.1'

    env['ASTRAL_WEB_MONITOR_HOST'] = '0.0.0.0'
    result = subprocess.run(
        [sys.executable, '-c', 'from astral_web_monitor.config import WEB_HOST; print(WEB_HOST)'],
        env=env, capture_output=True, text=True, check=True,
    )
    assert result.stdout.strip() == '0.0.0.0'


def test_ros_launch_defaults_to_loopback(monkeypatch):
    monkeypatch.delenv('ASTRAL_WEB_MONITOR_HOST', raising=False)
    path = PACKAGE / 'launch/web_monitor.launch.py'
    spec = importlib.util.spec_from_file_location('web_monitor_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    launch_description = module.generate_launch_description()
    arguments = {
        argument.name: argument.default_value
        for argument in launch_description.entities
        if isinstance(argument, DeclareLaunchArgument)
    }
    assert arguments['web_host'][0].text == '127.0.0.1'

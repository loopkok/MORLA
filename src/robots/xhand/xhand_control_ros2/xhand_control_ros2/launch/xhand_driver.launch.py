"""Explicit single-hand vendor driver launch; canonical stack uses MORLA bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    side = LaunchConfiguration("side").perform(context).strip().lower()
    port = LaunchConfiguration("serial_port").perform(context).strip()
    if side not in ("left", "right"):
        raise RuntimeError("side must be left or right")
    if not port:
        raise RuntimeError("serial_port must be explicitly configured for this host")
    return [
        Node(
            package="xhand_control_ros2",
            executable="xhand_control_ros2_node",
            name=f"xhand_control_{side}",
            namespace=f"{side}_hand/driver",
            output="screen",
            parameters=[{
                "port_name": port,
                "update_rate": 50.0,
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="left"),
        DeclareLaunchArgument("serial_port", default_value=""),
        OpaqueFunction(function=_setup),
    ])

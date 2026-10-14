#!/usr/bin/env python3
"""只起 MuJoCo 仿真节点（自己往话题里发命令即可）。

    ros2 launch xnero_mujoco_sim xnero_mujoco_sim.launch.py
    ros2 launch xnero_mujoco_sim xnero_mujoco_sim.launch.py enable_viewer:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("enable_viewer", default_value="true"),
        DeclareLaunchArgument("mjcf_path", default_value="",
                              description="空 = 从 robot_description asset manifest 定位"),
        DeclareLaunchArgument("publish_tactile", default_value="true"),
        DeclareLaunchArgument("state_rate", default_value="50.0"),
        DeclareLaunchArgument("realtime", default_value="true"),
    ]
    sim = Node(
        package="xnero_mujoco_sim",
        executable="xnero_mujoco_sim_node",
        name="xnero_mujoco_sim_node",
        output="screen",
        parameters=[{
            "enable_viewer": LaunchConfiguration("enable_viewer"),
            "mjcf_path": LaunchConfiguration("mjcf_path"),
            "publish_tactile": LaunchConfiguration("publish_tactile"),
            "state_rate": LaunchConfiguration("state_rate"),
            "realtime": LaunchConfiguration("realtime"),
        }],
    )
    return LaunchDescription(args + [sim])

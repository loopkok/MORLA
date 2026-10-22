"""Start operator inputs alongside any profile-driven robot; never enables motors."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    voice = value("voice_source")
    if voice not in {"off", "host", "quest", "both"}:
        raise ValueError("voice_source must be off, host, quest or both")
    config = value("config")
    actions = []
    if value("start_router").lower() == "true":
        actions.append(Node(package="morla_operator_control", executable="operator_router", output="screen"))
    for source in ("host", "quest"):
        if voice in {source, "both"}:
            actions.append(Node(package="morla_operator_control", executable="voice_input",
                name="morla_voice_" + source, output="screen", parameters=[config, {
                    "source": source, "dry_run": value("dry_run").lower() == "true",
                    "microphone_device": value("microphone_device"),
                }]))
    if value("pedal_device"):
        actions.append(Node(package="morla_operator_control", executable="pedal_input", output="screen",
                            parameters=[config, {"device": value("pedal_device"), "dry_run": value("dry_run").lower() == "true"}]))
    if value("feedback").lower() == "true":
        actions.append(Node(package="morla_operator_control", executable="operator_feedback", output="screen"))
    return actions


def generate_launch_description():
    config = os.path.join(get_package_share_directory("morla_operator_control"), "config", "operator_control.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("start_router", default_value="true"),
        DeclareLaunchArgument("voice_source", default_value="off"),
        DeclareLaunchArgument("microphone_device", default_value=""),
        DeclareLaunchArgument("pedal_device", default_value=""),
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("feedback", default_value="true"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        OpaqueFunction(function=setup),
    ])

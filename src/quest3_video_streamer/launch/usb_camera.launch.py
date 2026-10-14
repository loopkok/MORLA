"""Launch a USB webcam -> Quest 3 WebRTC push.

Starts:
  * usb_cam node (V4L2 USB camera, MJPG for high-res USB webcams)
  * quest3_video_streamer node subscribing to the usb_cam image topic

Two source modes are available via the ``source_type`` arg:
  * ``ros`` (default): usb_cam publishes sensor_msgs/Image; the streamer
    subscribes to it. Works when a ROS camera driver is desired.
  * ``webcam``: the streamer opens the USB camera directly with OpenCV (no
    usb_cam node needed). Use this for MJPG-only cheap cameras when usb_cam
    cannot decode MJPG at the target resolution.

Usage:
    ros2 launch quest3_video_streamer usb_camera.launch.py
    ros2 launch quest3_video_streamer usb_camera.launch.py video_device:=/dev/video6
    ros2 launch quest3_video_streamer usb_camera.launch.py source_type:=webcam webcam_index:=6
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext


def generate_launch_description():
    source_type = LaunchConfiguration('source_type')
    video_device = LaunchConfiguration('video_device')
    webcam_index = LaunchConfiguration('webcam_index')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    image_fps = LaunchConfiguration('image_fps')
    preset = LaunchConfiguration('preset')
    fov_h_deg = LaunchConfiguration('fov_h_deg')
    source_label = LaunchConfiguration('source_label')
    signaling_port = LaunchConfiguration('signaling_port')
    mocap_port = LaunchConfiguration('mocap_port')
    enable_mocap_tcp = LaunchConfiguration('enable_mocap_tcp')
    verbose = LaunchConfiguration('verbose')

    streamer_params = {
        'preset': preset,
        'fov_h_deg': fov_h_deg,
        'source_label': source_label,
        'signaling_host': '0.0.0.0',
        'signaling_port': signaling_port,
        'mocap_tcp_host': '0.0.0.0',
        'mocap_tcp_port': mocap_port,
        'enable_mocap_tcp': enable_mocap_tcp,
        'verbose': verbose,
    }

    def _build(context: LaunchContext):
        st = source_type.perform(context)
        nodes = []
        if st == 'webcam':
            # Direct OpenCV source: no usb_cam node needed.
            streamer_params_local = dict(streamer_params)
            streamer_params_local['source_type'] = 'webcam'
            streamer_params_local['webcam_index'] = int(webcam_index.perform(context))
            nodes.append(Node(
                package='quest3_video_streamer',
                executable='quest3_video_streamer',
                name='quest3_video_streamer',
                parameters=[streamer_params_local],
                output='screen',
            ))
        else:
            # ROS source: usb_cam publishes the image, streamer subscribes.
            image_topic = '/image_raw'
            usb_cam_node = Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam',
                parameters=[{
                'video_device': video_device.perform(context),
                'image_width': int(image_width.perform(context)),
                'image_height': int(image_height.perform(context)),
                'framerate': float(image_fps.perform(context)),
                'pixel_format': 'mjpeg2rgb',
                'io_method': 'mmap',
            }],
            output='screen',
        )
            streamer_params_local = dict(streamer_params)
            streamer_params_local['source_type'] = 'ros'
            streamer_params_local['image_topic'] = image_topic
            nodes.append(usb_cam_node)
            nodes.append(Node(
                package='quest3_video_streamer',
                executable='quest3_video_streamer',
                name='quest3_video_streamer',
                parameters=[streamer_params_local],
                output='screen',
            ))
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('source_type', default_value='webcam'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('webcam_index', default_value='0'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('image_fps', default_value='30'),
        DeclareLaunchArgument('preset', default_value='720p30'),
        # Typical USB webcam horizontal FOV ~60°. Set to your camera's real FOV
        # so the Quest panel is sized at natural scale (fixes magnification).
        DeclareLaunchArgument('fov_h_deg', default_value='60.0'),
        DeclareLaunchArgument('source_label', default_value='usb_cam'),
        DeclareLaunchArgument('signaling_port', default_value='8765'),
        DeclareLaunchArgument('mocap_port', default_value='8000'),
        # 默认不启用 mocap sink：与 quest3_hand_mocap 同时运行时让 hand_mocap 独占 8000。
        DeclareLaunchArgument('enable_mocap_tcp', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        GroupAction([OpaqueFunction(function=_build)]),
    ])

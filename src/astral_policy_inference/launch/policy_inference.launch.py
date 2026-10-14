"""astral_policy_inference launch: policy_node + optional policy_keyboard.

用法：
  ros2 launch astral_policy_inference policy_inference.launch.py \
      params_file:=<path> backend_type:=remote host:=192.168.x.x \
      camera_image_size:=480 keyboard:=true

backend_type = 传输（remote | inproc | stub）；model = 模型族（act | pi05 | ...），
默认回落 yaml。顶层 robot/schema 参数通过 `policy.robot.<key>` 形如
--ros-args -p arms:="['left']" 在 launch 里用 extra_args 传入（见 config yaml 注释）。

注意：模型 preprocessor 无 resize，`camera_image_size` 必须与模型输入一致
（本机 pickup_act_480 = 480）；launch 未传时回落 yaml 默认值 224。
`engine_mode` 默认 queue_async 即 30Hz（方案 A 后无需改）；仅时序融合 checkpoint
（`temporal_ensemble_coeff` 非空）才需 `engine_mode:=queue_sync`。
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# run 目录生成（生产路径用包内已单测的 runlog.run_log_dir；裸路径直接 launch
# 文件时包不在 sys.path，内联兜底，避免解析期 ModuleNotFoundError）。
try:
    from astral_policy_inference.runlog import run_log_dir
except ImportError:  # pragma: no cover - 生产安装路径走上面的 import
    def run_log_dir(root, tag=""):
        root = (root or "").strip()
        if not root:
            return ""
        tag = (tag or "").strip().replace("/", "_").replace(" ", "_")
        name = f"{time.strftime('%Y%m%d-%H%M%S')}_{tag}" if tag else time.strftime("%Y%m%d-%H%M%S")
        d = os.path.join(root, name)
        os.makedirs(d, exist_ok=True)
        return d


def _default_params() -> str:
    return os.path.join(
        get_package_share_directory("astral_policy_inference"),
        "config",
        "policy_inference.yaml",
    )


def _node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    backend_type = LaunchConfiguration("backend_type").perform(context)
    model = LaunchConfiguration("model").perform(context)
    jpeg_transport = LaunchConfiguration("jpeg_transport").perform(context)
    host = LaunchConfiguration("host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    checkpoint_dir = LaunchConfiguration("checkpoint_dir").perform(context)
    cmd_topic = LaunchConfiguration("cmd_topic").perform(context)
    camera_image_size = LaunchConfiguration("camera_image_size").perform(context)
    engine_mode = LaunchConfiguration("engine_mode").perform(context)
    control_interp = LaunchConfiguration("control_interp").perform(context)
    temporal_ensemble_coeff = LaunchConfiguration("temporal_ensemble_coeff").perform(context)
    chunk_anchor_tol = LaunchConfiguration("chunk_anchor_tol").perform(context)
    chunk_anchor_blend = LaunchConfiguration("chunk_anchor_blend").perform(context)
    async_prefetch_ahead = LaunchConfiguration("async_prefetch_ahead").perform(context)
    metrics_log_file = LaunchConfiguration("metrics_log_file").perform(context)
    joint_stream_log_file = LaunchConfiguration("joint_stream_log_file").perform(context)
    control_diagnostics_log_file = LaunchConfiguration(
        "control_diagnostics_log_file"
    ).perform(context)
    camera_diagnostics_log_file = LaunchConfiguration(
        "camera_diagnostics_log_file"
    ).perform(context)
    plan_trace_log_file = LaunchConfiguration("plan_trace_log_file").perform(context)
    log_dir = LaunchConfiguration("log_dir").perform(context)
    log_tag = LaunchConfiguration("log_tag").perform(context)
    # log_dir 模式：每次 launch 建 {log_dir}/{stamp}_{tag}/ 运行目录，metrics + cmd +
    # control + camera diagnostics 四条流一起落进去（不复用 /tmp 文件）。显式路径参数
    # 仍分别优先于 log_dir。
    run_dir = ""
    if log_dir:
        run_dir = run_log_dir(log_dir, log_tag)
    parameters = [params_file]
    if backend_type:
        params_bt = {"backend_type": backend_type}
        # 只 append 显式传入的键（host/port/checkpoint_dir 默认空串）——否则 launch
        # 默认值会静默覆盖 yaml（data_collect 不变量 1 的反模式；曾导致改了 yaml 的
        # host/port 却连到 127.0.0.1:8000）。cmd_topic 保留默认（keyboard 依赖且与 yaml 一致）。
        if host:
            params_bt["host"] = host
        if port:
            params_bt["port"] = int(port)
        if checkpoint_dir:
            params_bt["checkpoint_dir"] = checkpoint_dir
        if cmd_topic:
            params_bt["cmd_topic"] = cmd_topic
        parameters.append(params_bt)
    if model:
        parameters.append({"model": model})
    if jpeg_transport:
        parameters.append({"jpeg_transport": jpeg_transport.lower() in ("1", "true")})
    if camera_image_size:
        parameters.append({"camera_image_size": int(camera_image_size)})
    if engine_mode:
        parameters.append({"engine_mode": engine_mode})
    if control_interp:
        parameters.append({"control_interp": int(control_interp)})
    if temporal_ensemble_coeff:
        parameters.append({"temporal_ensemble_coeff": float(temporal_ensemble_coeff)})
    if chunk_anchor_tol:
        parameters.append({"chunk_anchor_tol": float(chunk_anchor_tol)})
    if chunk_anchor_blend:
        parameters.append({"chunk_anchor_blend": int(chunk_anchor_blend)})
    if async_prefetch_ahead:
        parameters.append({"async_prefetch_ahead": int(async_prefetch_ahead)})
    if metrics_log_file:
        parameters.append({"metrics_log_file": metrics_log_file})
    elif run_dir:
        parameters.append({"metrics_log_file": os.path.join(run_dir, "pi_metrics.jsonl")})
    if joint_stream_log_file:
        parameters.append({"joint_stream_log_file": joint_stream_log_file})
    elif run_dir:
        parameters.append({"joint_stream_log_file": os.path.join(run_dir, "pi_cmds.jsonl")})
    if control_diagnostics_log_file:
        parameters.append({
            "control_diagnostics_log_file": control_diagnostics_log_file
        })
    elif run_dir:
        parameters.append({
            "control_diagnostics_log_file": os.path.join(run_dir, "pi_control.jsonl")
        })
    if camera_diagnostics_log_file:
        parameters.append({
            "camera_diagnostics_log_file": camera_diagnostics_log_file
        })
    elif run_dir:
        parameters.append({
            "camera_diagnostics_log_file": os.path.join(
                run_dir, "camera_diagnostics.jsonl"
            )
        })
    if plan_trace_log_file:
        parameters.append({"plan_trace_log_file": plan_trace_log_file})
    elif run_dir:
        parameters.append({"plan_trace_log_file": os.path.join(run_dir, "pi_plan_trace.jsonl")})
    node = Node(
        package="astral_policy_inference",
        executable="policy_node",
        name="policy_node",
        output="screen",
        parameters=parameters,
    )
    kb = LaunchConfiguration("keyboard").perform(context).lower() in ("1", "true")
    actions = [node]
    if kb:
        actions.append(
            Node(
                package="astral_policy_inference",
                executable="policy_keyboard",
                name="policy_keyboard",
                output="screen",
                parameters=[{"cmd_topic": cmd_topic, "state_topic": LaunchConfiguration("state_topic")}],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=_default_params()),
            DeclareLaunchArgument("backend_type", default_value=""),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument(
                "jpeg_transport", default_value="",
                description="true 上行 camera 槽位发 JPEG（载荷 ~7x 小）；false 发 RGB"),
            DeclareLaunchArgument("host", default_value="",
                                  description="远程 serve 主机 IP（默认空=用 yaml）"),
            DeclareLaunchArgument("port", default_value="",
                                  description="远程 serve 端口（默认空=用 yaml）"),
            DeclareLaunchArgument("checkpoint_dir", default_value=""),
            DeclareLaunchArgument("camera_image_size", default_value=""),
            DeclareLaunchArgument("engine_mode", default_value=""),
            DeclareLaunchArgument(
                "control_interp", default_value="",
                description="控制率 = dataset_fps×N（2=60Hz 插值下发，拆小 30Hz 大步）"),
            DeclareLaunchArgument(
                "temporal_ensemble_coeff", default_value="",
                description=">0 开启 ACT 时序融合（换 chunk 加权平均，消切换跳变；0=关）"),
            DeclareLaunchArgument(
                "chunk_anchor_tol", default_value="",
                description=">0 换 chunk 切换平滑（续播起点偏离旧 command >tol 时前 blend 行过渡；0=关）"),
            DeclareLaunchArgument(
                "chunk_anchor_blend", default_value="",
                description="切换平滑过渡行数（配合 chunk_anchor_tol；@30Hz 每行 33ms）"),
            DeclareLaunchArgument(
                "async_prefetch_ahead", default_value="",
                description="重规划/融合重叠行数（每 action_chunk−N 步融合一次；0=自动 chunk//2）"),
            DeclareLaunchArgument(
                "metrics_log_file", default_value="",
                description="非空则节点把 state JSON（延迟/引擎指标）追加写该文件并终端打印"),
            DeclareLaunchArgument(
                "joint_stream_log_file", default_value="",
                description="非空则节点把每次下发的关节指令流（30Hz JSON 行）追加写该文件"),
            DeclareLaunchArgument(
                "control_diagnostics_log_file", default_value="",
                description=(
                    "非空则逐控制 tick 记录调度/执行耗时、观测龄期和 hold 原因"
                )),
            DeclareLaunchArgument(
                "camera_diagnostics_log_file", default_value="",
                description=(
                    "非空则逐帧记录 policy 图像接收/解码，并合并 streamer "
                    "capture/tap 分层统计"
                )),
            DeclareLaunchArgument(
                "plan_trace_log_file", default_value="",
                description="非空则记录每次 plan 的观测快照、完整 chunk 与续播对齐 JSONL"),
            DeclareLaunchArgument(
                "log_dir", default_value="",
                description=(
                    "非空根目录 → 每次 launch 自动建 {log_dir}/{YYYYMMDD-HHMMSS}[_tag]/ "
                    "运行目录，把 pi_metrics.jsonl + pi_cmds.jsonl + pi_control.jsonl + "
                    "camera_diagnostics.jsonl + pi_plan_trace.jsonl "
                    "落进去（区分每次记录、"
                    "持久化、不复用同一文件）。五个显式日志路径参数分别优先。"
                ),
            ),
            DeclareLaunchArgument(
                "log_tag", default_value="",
                description="事件名，追加到运行目录名（log_dir 模式）"),
            DeclareLaunchArgument("keyboard", default_value="false"),
            DeclareLaunchArgument("cmd_topic", default_value="/policy_inference/cmd"),
            DeclareLaunchArgument("state_topic", default_value="/policy_inference/state"),
            OpaqueFunction(function=_node),
        ]
    )

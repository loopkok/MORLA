"""One lazy CLI for the offline data pipeline; collection requires no export dependencies."""
import argparse
import importlib
import os
from pathlib import Path
import subprocess
import sys


COMMANDS = {
    "prepare": ("prepare_session", "复制原始数据到工作目录，可显式映射相机名称"),
    "preview": ("preview_session", "生成浏览器图像抽查与状态曲线"),
    "quantify": ("quantify_cmd_state", "分析 raw 的 cmd/state 或 aligned 动作"),
    "align": ("align_data", "raw → aligned_data.h5"),
    "validate": ("validate_data", "质量检查；--apply 隔离失败段"),
    "repair": ("repair_aligned", "保守清洗 aligned，输出新 session"),
    "replay": ("replay_rerun", "Rerun 回放 aligned"),
    "export-openpi": ("convert_to_lerobot", "aligned → LeRobot v2.1"),
    "export-act": ("convert_to_act", "session 或 v2.1 → ACT v3 + 自检"),
    "upgrade-v3": ("convert_to_lerobot_v3", "已有 v2.1 → v3"),
    "verify-aligned": ("verify_aligned", "检查清洗后的 state/图像/action 来源"),
    "verify-existing": ("verify_existing_data_pipeline", "已有 raw 的隔离样本端到端验证"),
    "migrate-cameras": ("migrate_camera_labels", "历史相机名称迁移工具，原地修改；先 --dry-run"),
    "process-openpi": (None, "一键：对齐 → 校验/隔离 → v2.1"),
    "process-act": (None, "一键：对齐 → 校验/隔离 → v3 + 自检"),
}


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        prog="data_pipeline", formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Astral 离线数据处理统一入口\n\n" + "\n".join(
            f"  {name:18} {description}" for name, (_, description) in COMMANDS.items()),
        epilog="各步骤参数：data_pipeline <command> --help\n清洗/诊断为独立可选步骤，一键流程不自动执行 repair。")
    parser.add_argument("command", choices=COMMANDS, nargs="?")
    # Parse only the command: the selected tool owns its arguments and help.
    parsed = parser.parse_args(args[:1])
    if parsed.command is None:
        parser.print_help()
        return 0
    module, _ = COMMANDS[parsed.command]
    if module is None:
        env = {**os.environ, "ASTRAL_DATA_PYTHON": sys.executable}
        return subprocess.run(["bash", str(Path(__file__).with_name("process.sh")),
                               parsed.command, *args[1:]], env=env).returncode
    tool = importlib.import_module(f"astral_data_collect.pipeline.{module}")
    previous = sys.argv
    try:
        sys.argv = [f"data_pipeline {parsed.command}", *args[1:]]
        return tool.main() or 0
    finally:
        sys.argv = previous

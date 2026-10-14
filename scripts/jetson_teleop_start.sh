#!/usr/bin/env bash
# ============================================================================
# Astral 遥操启动脚本（Jetson 终端1：清场 + adb + Web 控制台）
#
# 流程：
#   0. 终端环境归一 + sudo systemctl stop rob-station.target
#   1. 清场：停 daemon、查残留节点（有残留→提示确认后强清/退出）、
#      查 astral_drivers（抢 8081）、查 Web 端口（8080 被占→可换 8088）
#   2. adb：确认 Quest 在线（device），清掉旧 reverse 再重新建立
#   3. 前台启动 astral_web_monitor（本终端一直挂着）
#
# 用法：bash scripts/jetson_teleop_start.sh     （或加执行权限后直接运行）
# ============================================================================

set -uo pipefail  # 不用 -e：交互确认需要自己控制分支

WS=/home/nvidia/loopkok/astral_ws
WEB_PORT_DEFAULT=8080
WEB_PORT_FALLBACK=8088

info()  { printf '\033[1;34m[*]\033[0m %s\n' "$*"; }
ok()    { printf '\033[1;32m[✓]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[!]\033[0m %s\n' "$*"; }
err()   { printf '\033[1;31m[✗]\033[0m %s\n' "$*"; }
die()   { err "$*"; exit 1; }

confirm() {  # confirm "提示语" → 0=yes / 1=no（默认 no）
    local ans
    read -r -p "$(printf '\033[1;33m[?]\033[0m %s [y/N] ' "$1")" ans
    [[ "${ans:-}" =~ ^[yY]$ ]]
}

# ----------------------------------------------------------------------------
# 0. 终端环境（每个新终端的标准前奏）
# ----------------------------------------------------------------------------
info "终端环境归一（conda deactivate / PATH / ROS / 工作区）"
conda deactivate 2>/dev/null || true
export PATH=/usr/bin:$PATH
# ROS setup.bash 会读未导出的 AMENT_TRACE_SETUP_FILES 等变量；
# 脚本开了 set -u，必须先关掉 nounset 再 source。
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash || die "找不到 /opt/ros/humble/setup.bash"
# shellcheck disable=SC1091
source "$WS/install/setup.bash" || die "找不到 $WS/install/setup.bash（先 colcon build）"
set -u

info "停止 rob-station.target（释放 8080；sudo 会询问密码）"
if sudo systemctl stop rob-station.target; then
    ok "rob-station.target 已停止"
else
    warn "rob-station.target 停止失败（不存在/无权限/超时），继续"
fi

# ----------------------------------------------------------------------------
# 1. 清场
# ----------------------------------------------------------------------------
info "清场：停 ros2 daemon"
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

# 1a. 残留节点检查：没清干净 → 提示后再决定强清或退出
NODES=$(ros2 node list 2>/dev/null || true)
if [[ -n "$NODES" ]]; then
    warn "检测到残留 ROS 节点："
    echo "$NODES" | sed 's/^/      /'
    if confirm "强制清理这些节点进程？（No = 退出，手动排查后再启动）"; then
        pkill -f "astral_arm_teleop"      2>/dev/null || true
        pkill -f "ik_solver"              2>/dev/null || true
        pkill -f "quest3_udp_mocap"       2>/dev/null || true
        pkill -f "quest3_video_streamer"  2>/dev/null || true
        pkill -f "pinch_gripper_node"     2>/dev/null || true
        pkill -f "head_teleop_node"       2>/dev/null || true
        pkill -f "controller_start_gate"  2>/dev/null || true
        pkill -f "data_collect"           2>/dev/null || true
        pkill -f "keyboard_controller"    2>/dev/null || true
        pkill -f "wujihand"               2>/dev/null || true
        pkill -f "astral_web_monitor"     2>/dev/null || true
        pkill -f "astral_robot_driver"    2>/dev/null || true
        pkill -f "ros2 launch"            2>/dev/null || true
        sleep 2
        ros2 daemon stop >/dev/null 2>&1 || true
        sleep 1
        NODES=$(ros2 node list 2>/dev/null || true)
        [[ -z "$NODES" ]] || die "强清后仍有节点：$NODES —— 请手动排查"
        ok "残留节点已清干净"
    else
        die "已中止：请先手动清理残留节点再启动"
    fi
else
    ok "无残留节点（ros2 node list 为空）"
fi

# 1b. astral_drivers 会抢 driver 的 8081，本机不允许单独开
if pgrep -af "astral_drivers" | grep -v pgrep; then
    warn "astral_drivers 正在运行（会抢 8081 端口）"
    if confirm "杀掉 astral_drivers？"; then
        pkill -f "astral_drivers" || true
        sleep 1
        ok "astral_drivers 已停止"
    else
        die "已中止：本机不要单独开 astral_drivers"
    fi
else
    ok "无 astral_drivers（8081 未被抢）"
fi

# 1c. Web 端口：8080 被占（常见是 rob-station）→ 杀掉占用或换 8088
WEB_PORT=$WEB_PORT_DEFAULT
if ss -tln 2>/dev/null | grep -q ":${WEB_PORT_DEFAULT} "; then
    OWNER=$(ss -tlnp 2>/dev/null | grep ":${WEB_PORT_DEFAULT} " | head -1)
    warn "端口 ${WEB_PORT_DEFAULT} 被占用：$OWNER"
    if confirm "改到 ${WEB_PORT_FALLBACK} 启动 Web？（No = 退出，手动处理占用）"; then
        WEB_PORT=$WEB_PORT_FALLBACK
        if ss -tln 2>/dev/null | grep -q ":${WEB_PORT_FALLBACK} "; then
            die "备用端口 ${WEB_PORT_FALLBACK} 也被占用，手动排查后再启动"
        fi
    else
        die "已中止：请先释放 ${WEB_PORT_DEFAULT}（如停 rob-station 的 web）"
    fi
fi
ok "Web 端口：$WEB_PORT"

# 1d. 安全提示
warn "周围留空：Web 里一点「启动」，driver 会自动上电并走向 init_pose！"
confirm "确认机械臂周围安全？" || die "已中止：请确认安全后再启动"

# ----------------------------------------------------------------------------
# 2. Quest / adb（先清旧 reverse 再重新建立）
# ----------------------------------------------------------------------------
if ! command -v adb >/dev/null 2>&1; then
    warn "未找到 adb —— 跳过 adb 配置（Quest mocap 将不可用）"
else
    info "检查 Quest 连接（adb devices）"
    adb start-server >/dev/null 2>&1 || true
    DEV_STATE=$(adb devices | awk 'NR>1 && NF>=2 {print $2; exit}')
    if [[ "$DEV_STATE" != "device" ]]; then
        adb devices
        die "Quest 未就绪（状态: ${DEV_STATE:-无设备}）。请 USB 连接并允许调试后重跑"
    fi
    ok "Quest 已连接且授权"

    info "清理旧 adb reverse 映射"
    adb reverse --remove-all 2>/dev/null || true
    sleep 0.5
    LEFT=$(adb reverse --list 2>/dev/null | grep -c "tcp:" || true)
    [[ "$LEFT" == "0" ]] && ok "旧 reverse 已清空" || warn "reverse 残留：$(adb reverse --list)"

    info "建立 adb reverse tcp:8000（mocap HTS 通道）"
    adb reverse tcp:8000 tcp:8000 || die "adb reverse 8000 失败"
    adb reverse --list
    ok "adb reverse 就绪（8765 视频信令由 full_teleop 启动时自动建立）"

    echo
    echo "  >>> Quest 头显里 HTS：TCP / localhost / 8000，再 Start Stream <<<"
    echo "      （可先握柄，也可稍后；视频在 web 里勾选）"
fi

# ----------------------------------------------------------------------------
# 3. 启动 Web 控制台（前台，本终端一直挂着）
# ----------------------------------------------------------------------------
info "启动 astral_web_monitor（端口 $WEB_PORT）—— 本终端保持前台"
echo "      浏览器打开:  http://<jetson_ip>:$WEB_PORT"
echo
exec env ASTRAL_WEB_MONITOR_PORT=$WEB_PORT \
         ASTRAL_WEB_MONITOR_DIST=$WS/src/astral_web_monitor/web/dist \
         ros2 launch astral_web_monitor web_monitor.launch.py

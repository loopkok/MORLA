#!/usr/bin/env bash
# serve_policy.sh — GPU 主机统一策略 serve 启动器（scripts/serve.py 的包装）。
#
# 参数外置在 serve_policy.env（本脚本同目录），改参不动脚本；CLI 可临时覆盖：
#
#   ./serve_policy.sh [--config <file>] [--port N] [--stop] [--status] [--force]
#
#   start     （默认）启动 serve.py：先做两道冲突检查，再 nohup 后台起，
#             日志/pid 写入 LOG_DIR。
#   --stop    优雅停止"由本脚本启动"的 serve（按 pidfile，TERM 后等退）。
#   --status  查看：pidfile 里的进程是否活着 + 该端口当前被谁监听。
#   --force   start 时 pidfile 已有活进程 → 先优雅停旧再起新的（仍不杀无关进程）。
#   --port N  临时覆盖配置里的 PORT。
#
# 冲突检查（启动前必过）：
#   1. 端口已在 LISTEN（不管是不是我们起的）→ 拒绝并列出占用进程；
#   2. pidfile 存在且进程存活 → 已由本脚本在跑 → 报错（--force 才重启）。
# 结论：不会静默起两个 serve，也不会误杀别人的进程。
#
# 退出码：0=正常；1=冲突/参数/环境错误；2=未知参数。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
CFG="${SERVE_POLICY_CFG:-$SCRIPT_DIR/serve_policy.env}"

ACTION=start
FORCE=0
PORT_OVERRIDE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --config) CFG="${2:?--config 需要文件路径}"; shift ;;
        --port)   PORT_OVERRIDE="${2:?--port 需要端口}"; shift ;;
        --force)  FORCE=1 ;;
        --stop)   ACTION=stop ;;
        --status) ACTION=status ;;
        -h|--help) grep '^#' "$0" | grep -v '^#!' | sed 's/^# //; s/^#$//' ; exit 0 ;;
        *) echo "unknown arg: $1 (--help 看用法)" >&2; exit 2 ;;
    esac
    shift
done

# ---------------------------------------------------------------- 读配置
[ -f "$CFG" ] || {
    echo "缺少配置: $CFG" >&2
    echo "复制一份 serve_policy.env 到脚本同目录，或 --config <file> 指定。" >&2
    exit 1
}
set -a; source "$CFG"; set +a

PY="${PY:?配置缺 PY}"
MODEL="${MODEL:?配置缺 MODEL}"
CHECKPOINT_DIR="${CHECKPOINT_DIR:?配置缺 CHECKPOINT_DIR}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8001}"
[ -n "$PORT_OVERRIDE" ] && PORT="$PORT_OVERRIDE"
ACTION_DIM="${ACTION_DIM:-8}"
# 注意：默认值里绝不能放 JSON（含 {} 与转义引号）——bash 解析 ${VAR:-word} 时会把
# 词内的 } 当闭合符，变量已设也会被尾部字符污染（实测 SLOT_MAP 54→56 字符）。
# 所以 SLOT_MAP 走 env 必填（serve.py 也有默认，但为显式一致这里要求配置提供）。
SLOT_MAP="${SLOT_MAP:?配置缺 SLOT_MAP}"
DEFAULT_PROMPT="${DEFAULT_PROMPT:-}"
DEVICE="${DEVICE:-}"
POLICY_CONFIG="${POLICY_CONFIG:-pi05_astral}"
LOG_DIR="${LOG_DIR:-/tmp/astral_serve}"

SERVE="$WS_ROOT/src/astral_policy_inference/scripts/serve.py"
PIDFILE="$LOG_DIR/serve_${MODEL}_${PORT}.pid"
LOGFILE="$LOG_DIR/serve_${MODEL}_${PORT}.log"

# ---------------------------------------------------------------- 工具函数
pid_alive() { # pid 参数：进程存活？
    [ -n "$1" ] && kill -0 "$1" 2>/dev/null
}

read_pidfile() {
    [ -f "$PIDFILE" ] && cat "$PIDFILE" 2>/dev/null || true
}

port_listener() { # 监听 PORT 的进程描述（无则空）
    ss -ltnp "sport = :$PORT" 2>/dev/null | awk 'NR>1{print $6}' \
        | sed 's/users:((//; s/))//' | head -1
}

# 注意：ss 无匹配时退出码仍是 0，必须按输出行判断（否则 port_busy 恒真、永远拒绝启动）
port_busy() { [ -n "$(ss -ltnH "sport = :$PORT" 2>/dev/null)" ]; }

graceful_stop() { # $1=pid：TERM 后最多等 10s，超时 KILL
    local pid="$1" n=0
    kill "$pid" 2>/dev/null || return 0
    while kill -0 "$pid" 2>/dev/null && [ $n -lt 20 ]; do
        sleep 0.5; n=$((n + 1))
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "serve 未在 10s 内退出，强制 KILL $pid" >&2
        kill -9 "$pid" 2>/dev/null || true
    fi
}

# ---------------------------------------------------------------- --status
if [ "$ACTION" = status ]; then
    echo "== serve_policy status: MODEL=$MODEL PORT=$PORT =="
    echo "config : $CFG"
    echo "log    : $LOGFILE"
    if port_busy; then
        echo "port $PORT : LISTEN by [$(port_listener)]"
    else
        echo "port $PORT : free"
    fi
    p="$(read_pidfile)"
    if [ -n "$p" ] && pid_alive "$p"; then
        echo "pidfile: $p (alive; started $(ps -o lstart= -p "$p" 2>/dev/null | tr -s ' '))"
    elif [ -n "$p" ]; then
        echo "pidfile: $p (dead/stale)"
    else
        echo "pidfile: none"
    fi
    exit 0
fi

# ---------------------------------------------------------------- --stop
if [ "$ACTION" = stop ]; then
    p="$(read_pidfile)"
    if [ -z "$p" ] || ! pid_alive "$p"; then
        echo "没有由本脚本启动的 serve 进程（pidfile=$PIDFILE 无/已死）"
        if port_busy; then
            echo "但端口 $PORT 仍被 [$(port_listener)] 占用——那是别的进程，勿用本脚本停。" >&2
            exit 1
        fi
        rm -f "$PIDFILE"
        exit 0
    fi
    echo "stopping serve pid=$p ..."
    graceful_stop "$p"
    rm -f "$PIDFILE"
    if port_busy; then
        echo "警告：端口 $PORT 仍被监听——可能还有别的 serve 残留。" >&2
        exit 1
    fi
    echo "stopped (port $PORT free)"
    exit 0
fi

# ---------------------------------------------------------------- start：校验
[ -x "$PY" ] || { echo "PY 不是可执行 python: $PY（改 serve_policy.env）" >&2; exit 1; }
[ -f "$SERVE" ] || { echo "serve.py 不存在: $SERVE" >&2; exit 1; }
[ -d "$CHECKPOINT_DIR" ] || {
    echo "checkpoint 目录不存在: $CHECKPOINT_DIR（改 serve_policy.env 的 CHECKPOINT_DIR）" >&2
    exit 1
}
mkdir -p "$LOG_DIR"

# 冲突检查：先按 pidfile 认"是不是我们的进程"——是我们 → --force 才停；不是/停完
# 端口仍忙 → 才是"他人占用" → 拒绝。不会静默双开，也不会误杀无关进程。
p="$(read_pidfile)"
if [ -n "$p" ] && pid_alive "$p"; then
    if [ "$FORCE" = 1 ]; then
        echo "发现本脚本旧实例 pid=$p，--force 先优雅停旧再启动 ..."
        graceful_stop "$p"
    else
        echo "serve 已在跑（pid=$p，LOG=$LOGFILE）；要重启加 --force，或先 --stop。" >&2
        exit 1
    fi
fi
if port_busy; then
    # 自己刚停的进程端口还没释放 → 等最多 10s；仍忙 → 是别的进程 → 拒绝
    n=0
    while port_busy && [ $n -lt 20 ]; do sleep 0.5; n=$((n + 1)); done
    if port_busy; then
        echo "端口冲突: $PORT 仍被 [$(port_listener)] 占用（非本脚本进程）" >&2
        echo "  - 若是别的 serve/服务：换 --port N 或手动处理占用方" >&2
        exit 1
    fi
fi

# ---------------------------------------------------------------- start：拉起
ARGS=(--model "$MODEL" --checkpoint-dir "$CHECKPOINT_DIR" --host "$HOST" --port "$PORT"
      --action-dim "$ACTION_DIM" --policy-config "$POLICY_CONFIG")
[ -n "$DEFAULT_PROMPT" ] && ARGS+=(--default-prompt "$DEFAULT_PROMPT")
[ -n "$DEVICE" ] && ARGS+=(--device "$DEVICE")
ARGS+=(--slot-map "$SLOT_MAP")

echo "=== 启动 serve[$MODEL] on ws://$HOST:$PORT ==="
echo "    checkpoint: $CHECKPOINT_DIR"
echo "    python    : $PY"
echo "    log       : $LOGFILE"

nohup "$PY" "$SERVE" "${ARGS[@]}" >"$LOGFILE" 2>&1 &
newpid=$!
echo "$newpid" > "$PIDFILE"

# 等端口起来（serve 加载 checkpoint 可能要几十秒）；给个进度提示
n=0
until port_busy || ! kill -0 "$newpid" 2>/dev/null; do
    [ $((n % 10)) -eq 0 ] && echo "  加载中 ... $((n))s (tail -f $LOGFILE)"
    sleep 1; n=$((n + 1))
    [ $n -ge 120 ] && { echo "120s 未就绪，见 $LOGFILE" >&2; exit 1; }
done

if ! kill -0 "$newpid" 2>/dev/null; then
    echo "serve 启动即退出（看日志）：" >&2
    tail -20 "$LOGFILE" >&2
    rm -f "$PIDFILE"
    exit 1
fi

echo "serve 就绪: ws://$HOST:$PORT (pid=$newpid)"
echo "  日志: tail -f $LOGFILE   停止: ./serve_policy.sh --stop"

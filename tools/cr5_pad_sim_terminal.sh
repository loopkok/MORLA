#!/usr/bin/env bash
# Desktop-icon wrapper: run the CR5 pad bench and keep the log window open.
# Kept separate so the .desktop Exec line needs no shell quoting.
set -u
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
"$DIR/start_cr5_pad_sim.sh"
status=$?
echo
echo "会话已结束（退出码 $status）。按回车关闭窗口。"
read -r

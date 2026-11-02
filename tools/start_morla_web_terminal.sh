#!/usr/bin/env bash
# Desktop-icon wrapper: run the web launcher and keep the log window open.
# Kept separate so the .desktop Exec line needs no shell quoting.
set -u
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
"$DIR/start_morla_web.sh"
status=$?
echo
echo "MORLA 控制台已结束（退出码 $status）。按回车关闭窗口。"
read -r

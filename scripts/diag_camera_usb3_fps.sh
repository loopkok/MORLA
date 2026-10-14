#!/usr/bin/env bash
# 诊断：同款 WN 相机在 USB3 上只有 120fps + JPEG 熵损坏，USB2 上可选 30fps 无损坏。
#
# 两个目的：
#   1. 判定损坏根源是「帧率 120fps」还是「USB3 端口」（对照实验，各录一段再 ffmpeg 数 error dc）
#   2. 测 USB3 相机能否被 VIDIOC_S_PARM 压到 30fps（枚举里没有 30，但不少 UVC 固件接受非枚举间隔）
#
# 用法（在 Jetson 上，改设备号按需）：
#   bash diag_camera_usb3_fps.sh /dev/video9 /dev/video0
#   参数1 = USB3 相机节点（默认 /dev/video9），参数2 = USB2 相机节点（默认 /dev/video0）
#
# 输出：各段录制的 mjpg 存 /tmp/，并打印 corruption count（ffmpeg "error dc" 计数）。
#       判定标准见末尾 VERDICT 段。
set -u

USB3_DEV="${1:-/dev/video9}"
USB2_DEV="${2:-/dev/video0}"
OUTDIR=/tmp/cam_diag
mkdir -p "$OUTDIR"

echo "=============================================="
echo "0) 控制列表（找帧率/带宽类控件，若 V4L2_CID_FRAME_RATE 等存在可直接定帧率）"
echo "=============================================="
v4l2-ctl -d "$USB3_DEV" -L 2>&1 | grep -iE "frame|rate|fps|interval|bandwidth|stream" || echo "(无相关控件)"
v4l2-ctl -d "$USB2_DEV" -L 2>&1 | grep -iE "frame|rate|fps|interval|bandwidth|stream" || echo "(无相关控件)"

echo ""
echo "=============================================="
echo "A) USB3 相机: 尝试 VIDIOC_S_PARM 压到 30fps"
echo "=============================================="
v4l2-ctl -d "$USB3_DEV" --set-fmt-video=width=1280,height=720,pixelformat=MJPG
v4l2-ctl -d "$USB3_DEV" --set-parm=30
echo -n "USB3 set-parm=30 后实际: "
v4l2-ctl -d "$USB3_DEV" --get-parm 2>&1 | grep -oE "Frames per second: [0-9.]+" || v4l2-ctl -d "$USB3_DEV" --get-parm

echo ""
echo "=============================================="
echo "B) USB3 录制（若 A 接受 30 则 30fps，否则 120fps；450 帧 @30=15s / @120≈3.75s）"
echo "=============================================="
v4l2-ctl -d "$USB3_DEV" --set-parm=30 \
  --stream-mmap --stream-count=450 --stream-to="$OUTDIR/usb3_after_setparm30.mjpg"
echo -n "实际帧数: "
v4l2-ctl -d "$USB3_DEV" --get-parm 2>&1 | grep -oE "Frames per second: [0-9.]+"
ls -la "$OUTDIR/usb3_after_setparm30.mjpg"

echo ""
echo "=============================================="
echo "C) USB2 相机 @120fps（关键对照：同 120fps 下 USB2 端口是否也损坏？）"
echo "=============================================="
v4l2-ctl -d "$USB2_DEV" --set-fmt-video=width=1280,height=720,pixelformat=MJPG
v4l2-ctl -d "$USB2_DEV" --set-parm=120
echo -n "USB2 set-parm=120 后实际: "
v4l2-ctl -d "$USB2_DEV" --get-parm 2>&1 | grep -oE "Frames per second: [0-9.]+" || v4l2-ctl -d "$USB2_DEV" --get-parm
v4l2-ctl -d "$USB2_DEV" --set-parm=120 \
  --stream-mmap --stream-count=1200 --stream-to="$OUTDIR/usb2_at120.mjpg"
ls -la "$OUTDIR/usb2_at120.mjpg"

echo ""
echo "=============================================="
echo "D) corruption 统计（error dc = 真实熵损坏；APP fields 报错是良性噪音，忽略）"
echo "=============================================="
for f in "$OUTDIR/usb3_after_setparm30.mjpg" "$OUTDIR/usb2_at120.mjpg"; do
  [ -f "$f" ] || continue
  dc=$(ffmpeg -hide_banner -loglevel error -f mjpeg -i "$f" -f null - 2>&1 | grep -c "error dc")
  ovr=$(ffmpeg -hide_banner -loglevel error -f mjpeg -i "$f" -f null - 2>&1 | grep -c "overread")
  frames=$(ffprobe -f mjpeg -select_streams v -show_entries stream=nb_read_frames -of csv=p=0 "$f" 2>/dev/null || echo "?")
  echo "  $f: error_dc=$dc overread=$ovr (帧数约 $frames)"
done

echo ""
echo "=============================================="
echo "E) USB 传输层错误（无 = 损坏是相机编码器自己产生，不是 USB 丢字节）"
echo "=============================================="
dmesg 2>/dev/null | grep -iE "uvc|usb .*error|xhci.*err|urb.*err" | tail -10 || echo "(dmesg 不可读，忽略)"

echo ""
echo "=============================================="
echo "VERDICT 判定"
echo "=============================================="
echo "1) 如果 B 段 get-parm 显示 30fps 且 error_dc=0 → 相机接受 USB3@30，直接按 30fps 用即可。"
echo "2) 如果 C 段 usb2_at120 error_dc 也 >0 → 损坏来自 120fps 本身（传感器/编码器吃不消），"
echo "   而非 USB3 端口。则 USB3 上必须 30fps：优先 USB2 线/USB2 hub（强制 HS 枚举→相机报 30fps），"
echo "   或确认 S_PARM=30 是否被接受。"
echo "3) 如果 C 段 clean、B 段 120fps+损坏 → 损坏与端口相关，USB3@30 也未必干净，"
echo "   优先把该相机挪到 USB2 链路。"
echo "4) 若 D 段 USB3 文件 error_dc=0 → 当前连接不损坏，直接用现配置。"
echo ""
echo "提示：USB2 线插 USB3 口（或 USB2-only hub）会强制 480Mbps HS 链路，相机改用其"
echo "HS 描述符 → 枚举回 30/60/120fps，720p30 即可用（带宽 ~6MB/s 远低于 USB2 上限）。"
echo "这不是改相机硬件，只是换线/换转接。"

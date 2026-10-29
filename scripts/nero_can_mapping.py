"""Read-only new-host CAN map validation; emits mapping only after both sides match."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--config', required=True)
p.add_argument('--status', action='store_true')
a = p.parse_args()
config_path = Path(a.config)
config = json.loads(config_path.read_text())
detected = []
for net in sorted(Path('/sys/class/net').iterdir()):
    if not (net/'type').exists() or (net/'type').read_text().strip() != '280':
        continue
    result = subprocess.run(['ethtool', '-i', net.name], capture_output=True, text=True)
    fields = dict(line.split(': ', 1) for line in result.stdout.splitlines() if ': ' in line)
    detected.append({'interface': net.name, 'bus_info': fields.get('bus-info', '')})
ports = {side: config.get(side + '_bus_info') for side in ('left', 'right')}
if a.status:
    print(json.dumps({'config': str(config_path), 'ports': ports, 'detected': detected}, ensure_ascii=False, indent=2))
    sys.exit(0)
if config.get('bitrate') != 1000000:
    sys.exit('Nero CAN bitrate must remain 1000000; no interface changed')
if any(not isinstance(v, str) or not re.fullmatch(r'\d+-\d+(?:\.\d+)*:\d+\.\d+', v) for v in ports.values()):
    sys.exit('新主机左右 CAN USB 映射尚未配置。接入设备后执行 bash scripts/can_muti_activate.sh --status，沿线确认左右臂，再填写 ' + str(config_path) + '。未修改任何接口。')
if ports['left'] == ports['right']:
    sys.exit('左右 CAN 不能使用同一 USB 端口；未修改任何接口')
for side, port in ports.items():
    matches = [row for row in detected if row['bus_info'] == port]
    if len(matches) != 1:
        sys.exit(f'{side} USB-CAN {port} 未唯一匹配；未修改任何接口。实际：{detected}')
    target = 'can_nero_' + side
    if any(row['interface'] == target and row['bus_info'] != port for row in detected):
        sys.exit(f'{target} 已被另一适配器占用；未修改任何接口')
for side, port in ports.items():
    print(port + '\tcan_nero_' + side)

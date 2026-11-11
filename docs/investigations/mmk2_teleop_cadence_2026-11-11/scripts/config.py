"""从 run manifest 里取出与节奏/手感相关的配置，供几次运行横向对照。

用法:
    /usr/bin/python3 scripts/config.py ~/.local/share/morla/logs/<run_dir> [...]

输出:
  quest      Quest 输入实例参数（关键是 side: left/right/both，它决定 APK 每帧发几只手）
  device     设备实例参数（FR3/CR5 的 max_velocity_rad_s、max_command_lead_rad 等）
  binding    每个绑定的控制器及其参数（position_scale、smoothing_alpha、
             max_joint_step、max_joint_velocity、command_lead_rad 等）
"""
import json
import sys

SCALAR = (int, float, bool, str)


def small(parameters):
    kept = {}
    for key, value in (parameters or {}).items():
        if isinstance(value, SCALAR):
            kept[key] = value
        elif isinstance(value, list) and len(value) <= 8 and not any(isinstance(v, list) for v in value):
            kept[key] = [round(v, 4) if isinstance(v, float) else v for v in value]
        elif isinstance(value, list):
            # 基变换矩阵之类：只留尺寸，避免把 3x3 数值铺满一行
            kept[key] = '<%dx%d>' % (len(value), len(value[0]) if value and isinstance(value[0], list) else 0)
    return kept


def report(run):
    manifest = json.load(open(run.rstrip('/') + '/manifest.json'))
    system = manifest.get('resolved_system') or {}
    print(run.rstrip('/').split('/')[-1])
    for instance in system.get('instances') or []:
        name = instance.get('instance_id') or instance.get('key') or ''
        parameters = small(instance.get('parameters'))
        if not parameters:
            continue
        if name == 'quest' or 'quest' in name:
            print('  quest    %s' % parameters)
        elif instance.get('kind') == 'actuator':
            print('  device   %-18s %s' % (name, parameters))
    for binding in system.get('bindings') or []:
        parameters = small(binding.get('parameters'))
        print('  binding  %-22s -> %-24s %-30s %s'
              % (binding.get('source'), binding.get('target'), binding.get('controller'), parameters))
    print()


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        report(argument)

"""Static layer-boundary checks; executable in a separate interpreter without ROS."""
import ast
import json
from pathlib import Path

root=Path(__file__).resolve().parents[3]
checks={}
def tree(relative):return ast.parse((root/relative).read_text())
def method(relative,name):
    return next(node for node in ast.walk(tree(relative)) if isinstance(node,ast.FunctionDef) and node.name==name)

runtime='src/common/morla_runtime/morla_runtime/session.py'
start=ast.unparse(method(runtime,'start'))
checks['runtime_has_no_native_brand_switch']=all(name not in start for name in ('morla.nero','morla.astral','morla.wuji','morla.xhand','make_native_adapters'))
engine=ast.unparse(method('src/common/morla_teleop/morla_teleop/engine.py','targets'))
checks['target_dispatch_has_no_input_or_robot_brand_pair']=all(name not in engine for name in ('vader','quest','astral','nero','epg'))
for file in ['src/common/morla_vader5_input/launch/vader5_teleop.launch.py',
             'src/robots/dobot/morla_cr5_driver/launch/cr5_vr.launch.py']:
    calls=[n.func.id for n in ast.walk(tree(file)) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
    checks[file+':no_duplicate_input_node']=not {'Node','IncludeLaunchDescription'}&set(calls)
shim=tree('src/common/morla_runtime/morla_runtime/native_ros.py')
checks['native_compatibility_is_import_only']=not any(isinstance(n,(ast.ClassDef,ast.FunctionDef)) for n in shim.body)
checks['legacy_and_unified_share_filters']='morla_teleop.gripper_processing' in (root/'src/robots/astral/astral_gripper_teleop/astral_gripper_teleop/pinch_gripper_node.py').read_text()
checks['kinematics_selected_by_plugin_metadata']=all(name not in ast.unparse(method('src/common/morla_teleop/morla_teleop/kinematics.py','solver_for')) for name in ('morla.nero','morla.astral','morla.dobot'))
print(json.dumps({'checks':checks,'hardware_accessed':False,'all_passed':all(checks.values())},indent=2))
if not all(checks.values()):raise SystemExit(1)

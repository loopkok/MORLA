"""Read measured joints and check local URDF FK/IK. Never enables or moves."""
import argparse
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .kinematics import pose_from_controller
from .urdf_ik import IKSolver


def probe(transport, solver=None, *, compare_controller_fk=False):
    solver=solver or IKSolver()
    q=np.asarray(transport.read()['values'],dtype=float)
    solver.sync_state(q)
    target=solver.fk(q)
    inverse=solver.solve(target)
    passed=inverse is not None
    actual=solver.fk(inverse) if passed else target
    position_error=float(np.linalg.norm(target[:3,3]-actual[:3,3]))
    angle_error=float(Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude())
    report={'read_only':True,'motion_commands_sent':False,'passed':passed,
            'solver':'cr5_urdf_pinocchio_dls','model_sha256':getattr(solver,'model_sha256',None),
            'measured_joints_rad':q.tolist(),'model_base_from_Link6':target.tolist(),
            'inverse_joints_rad':inverse.tolist() if passed else None,
            'position_error_m':position_error,'angle_error_rad':angle_error,
            'max_seed_error_rad':float(np.max(np.abs(inverse-q))) if passed else None,
            'physical_base_tcp_verified':False}
    if compare_controller_fk:
        # Diagnostic only: base/tool definitions may differ. Never changes the
        # local solver mapping or claims physical calibration from this number.
        remote=pose_from_controller(transport.forward_kinematics(q))
        report['controller_fk_comparison']={
            'position_error_m':float(np.linalg.norm(remote[:3,3]-target[:3,3])),
            'angle_error_rad':float(Rotation.from_matrix(remote[:3,:3]@target[:3,:3].T).magnitude()),
            'frame_alignment_assumed_for_diagnostic_only':True}
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host')
    parser.add_argument('--device-id')
    parser.add_argument('--offline',action='store_true')
    parser.add_argument('--joints',type=float,nargs=6,default=[0]*6,metavar='RAD')
    parser.add_argument('--compare-controller-fk',action='store_true')
    args=parser.parse_args()
    if args.offline:
        if args.compare_controller_fk:parser.error('--compare-controller-fk requires a real controller')
        from types import SimpleNamespace
        report=probe(SimpleNamespace(read=lambda:{'values':args.joints}))
        report['offline']=True
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 0 if report['passed'] else 2
    if not args.host or not args.device_id:parser.error('--host and --device-id required unless --offline')
    from morla_runtime.resource_manager import ResourceManager
    from .transport import CR5Transport
    resources=ResourceManager();transport=None
    try:
        resources.acquire(['morla:active-system','cr5:'+args.device_id])
        transport=CR5Transport({'host':args.host,'timeout_s':.2})
        report=probe(transport,compare_controller_fk=args.compare_controller_fk)
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 0 if report['passed'] else 2
    finally:
        try:
            if transport is not None:transport.close()
        finally:resources.release()

import numpy as np
import pytest
from dataclasses import replace
from morla_input.adapters import MobileAdapter
from morla_teleop.controllers import ControllerRegistry,BindingController,ControllerError
from morla_teleop.kinematics import CartesianController


class Solver:
    def sync_state(self,seed): self.seed=np.asarray(seed)
    def fk(self,seed):
        pose=np.eye(4);pose[:3,3]=seed[:3];return pose
    def solve(self,target):
        result=self.seed.copy();result[:3]=target[:3,3];return result


class BranchSolver(Solver):
    def __init__(self): self.resets=[]
    def sync_state(self,seed,reset_branch=True):
        self.resets.append(bool(reset_branch)); self.seed=np.asarray(seed)


class ReachSolver(Solver):
    """Solver whose reach ends at x=0.5, like a joint/wrist limit."""
    def __init__(self): self.failures=0
    def solve(self,target):
        if target[0,3] > .5:
            self.failures+=1
            return None
        return super().solve(target)


def test_unreachable_jog_skips_tick_and_keeps_session_armed():
    # A reach/limit miss is a normal jog event. The legacy teleop loops only
    # skipped the tick; raising ControllerError here revoked the whole
    # session, so the operator had to prepare/activate again mid-motion.
    solver=ReachSolver();controller=CartesianController(solver,jog=True)
    parameters={'axes_map':{'x':'x','y':'y'},'linear_speed':.1}
    seed=[0,0,0,0,0,0,0]
    forward={'x':1,'y':0}; backward={'x':-1,'y':0}
    for _ in range(10): assert controller(forward,seed,.02,parameters) is not None
    for _ in range(300): controller(forward,seed,.02,parameters)
    assert solver.failures==1
    held=controller.jog_target.copy()
    # Holding the stick against the limit must not wind the target further out
    # nor re-run the ~120 ms failed search every frame.
    for _ in range(50): controller(forward,seed,.02,parameters)
    assert solver.failures==1
    np.testing.assert_allclose(controller.jog_target,held)
    # Reversing the stick moves on the very next tick, without a reset.
    assert controller(backward,seed,.02,parameters) is not None


def test_branch_reset_only_on_first_solve_after_reset():
    # A cold branch search costs ~100 ms; repeating it every tick starves the
    # 50 Hz input queue. Only the first solve (or the first after a re-anchor)
    # may reset the continuity branch.
    solver=BranchSolver();controller=CartesianController(solver,jog=True)
    parameters={'axes_map':{'x':'x','y':'y'}}
    for _ in range(3): controller({'x':0,'y':0},[0]*7,.02,parameters)
    assert solver.resets==[True,False,False]
    controller.reset()
    controller({'x':0,'y':0},[0]*7,.02,parameters)
    assert solver.resets==[True,False,False,True]


def test_nonidentity_tool_does_not_move_neutral_jog():
    tool=np.eye(4);tool[2,3]=.12
    controller=CartesianController(Solver(),jog=True)
    seed=[.1,.2,.3,0,0,0,0]
    target=controller({'x':0,'y':0},seed,.02,
        {'axes_map':{'x':'x','y':'y'},'solver_tip_from_tool':tool.tolist()})
    np.testing.assert_allclose(target,seed)


def test_explicit_seed_integrates_from_the_last_command():
    # The CR5 control loop runs at ~90 Hz but its 30004 feedback arrives at
    # ~25 Hz. Re-seeding the solver from the measurement every tick holds the
    # command until a new sample lands, then jumps. A caller that holds the
    # commanded joints passes them as the integration seed; the measurement is
    # still used for the reported pose and the lead limits.
    class SeedSolver(Solver):
        def __init__(self): self.synced=[]; self.solved=[]
        def sync_state(self,seed): self.synced.append(np.asarray(seed,dtype=float)); super().sync_state(seed)
        def solve(self,target,seed=None):
            self.solved.append(None if seed is None else np.asarray(seed,dtype=float))
            return super().solve(target)
    solver=SeedSolver();controller=CartesianController(solver)
    measured=np.zeros(7);command=np.array([.3,0,0,0,0,0,0])
    controller.reanchor({'position':[0,0,0],'quaternion':[0,0,0,1]},measured,{})
    controller({'position':[.01,0,0],'quaternion':[0,0,0,1]},measured,.02,{},seed=command)
    np.testing.assert_allclose(solver.synced[-1],command)
    np.testing.assert_allclose(solver.solved[-1],command)
    np.testing.assert_allclose(controller.diagnostics()['feedback_seed'],measured)
    with pytest.raises(ControllerError):
        controller({'position':[.01,0,0],'quaternion':[0,0,0,1]},measured,.02,{},seed=[0,0,0])


def test_reanchor_and_source_identity_and_input_gap():
    clock=[100];source=MobileAdapter('phone',clock=lambda:clock[0]);source.reconnect()
    binding={'source':'phone.scalar','target':'tool.closure','deadman':'phone.hold_to_control','controller':'morla.scalar_map'}
    controller=BindingController(binding,['g'],clock=lambda:clock[0])
    def frame(index,down):return source.packet({'epoch':source.epoch,'sequence':index,'hold_to_control':down,'scalar':.4})
    assert controller.step(frame(0,False),[0])==[.4]
    assert controller.step(frame(1,True),[0])==[.4]
    clock[0]+=300_000_000
    assert controller.step(frame(2,True),[0])==[.4] # A gap resets the anchor but does not block.
    with pytest.raises(ControllerError):controller.step(replace(frame(3,False),source_id='other'),[0])


def test_per_port_frame_does_not_reanchor_on_unrelated_landmarks():
    clock=[100];source=MobileAdapter('phone',clock=lambda:clock[0]);source.reconnect()
    binding={'source':'phone.scalar','target':'tool.closure','deadman':'phone.hold_to_control','controller':'morla.scalar_map'}
    controller=BindingController(binding,['g'],clock=lambda:clock[0])
    def f(index,down,frame):
        x=source.packet({'epoch':source.epoch,'sequence':index,'hold_to_control':down,'scalar':.4})
        return replace(x,frame_id=frame,port_metadata={'scalar':{'frame_id':'control','received_ns':clock[0]}})
    assert controller.step(f(0,False,'hand_local'),[0])==[.4]
    assert controller.step(f(1,True,'body'),[0])==[.4]


def test_sdl_button_positions_are_not_legacy_quest_positions():
    from morla_input.adapters import GamepadAdapter
    pad=GamepadAdapter('pad');pad.reconnect();buttons=[0]*21;buttons[10]=1
    frame=pad.joy([0]*6,buttons)
    assert frame.ports['right_shoulder'] is True
    assert frame.ports['guide'] is False


def test_explicit_anchor_ack_uses_fk_without_solving_or_dispatching():
    class CountSolver(Solver):
        calls=0
        def solve(self,target):self.calls+=1;return super().solve(target)
    solver=CountSolver();controller=CartesianController(solver)
    seed=[.1,.2,.3,0,0,0,0]
    value={'position':[1.,2.,3.],'quaternion':[0,0,0,1]}
    ack=controller.reanchor(value,seed,{})
    assert ack['acknowledged'] and solver.calls==0
    np.testing.assert_allclose(np.asarray(ack['current_tcp'])[:3,3],seed[:3])
    moved={'position':[1.01,2.,3.],'quaternion':[0,0,0,1]}
    np.testing.assert_allclose(controller(moved,seed,.02,{})[:3],[.11,.2,.3])


def test_binding_anchor_identity_survives_the_first_fresh_manual_frame():
    from morla_input.adapters.normalized import InputFrame
    clock=[1_000_000_000]
    registry=ControllerRegistry();plugin=CartesianController(Solver())
    registry.register('morla.cartesian_delta_ik',plugin)
    binding={'id':'pose','source':'quest.right_pose','target':'arm.position_target',
             'controller':'morla.cartesian_delta_ik','parameters':{}}
    controller=BindingController(binding,[f'j{i}' for i in range(7)],registry=registry,clock=lambda:clock[0])
    def frame(seq,x):
        return InputFrame('quest','epoch',seq,clock[0],'world',
            {'right_pose':{'position':[x,2.,3.],'quaternion':[0,0,0,1]}},
            port_metadata={'right_pose':{'received_ns':clock[0],'frame_id':'world/right/controller'}})
    seed=[.1,.2,.3,0,0,0,0]
    assert controller.reanchor(frame(1,1.),seed)['acknowledged']
    clock[0]+=20_000_000
    np.testing.assert_allclose(controller.step(frame(2,1.01),seed)[:3],[.11,.2,.3])


@pytest.mark.parametrize('matrix,expected_per_axis',[
    # det +1 rotation (the CR5/legacy shape) and det -1 basis change (shipped FR3).
    ([[0,0,1],[1,0,0],[0,1,0]],[[0,.025,0],[0,0,.025],[.025,0,0]]),
    ([[-1,0,0],[0,0,-1],[0,1,0]],[[-.025,0,0],[0,0,.025],[0,-.025,0]]),
])
def test_independent_position_mapping_preserves_orientation(matrix,expected_per_axis):
    from scipy.spatial.transform import Rotation
    rotation=np.array([[0,0,-1],[1,0,0],[0,1,0]])
    parameters={'input_rotation':rotation.tolist(),
                'input_position_rotation':matrix,'position_scale':.25}
    for axis,expected in enumerate(expected_per_axis):
        controller=CartesianController(Solver());measured=np.zeros(7)
        controller.reanchor({'position':[0,0,0],'quaternion':[0,0,0,1]},measured,parameters)
        position=np.eye(3)[axis]*.1
        delta=Rotation.from_rotvec([.05,0,0])
        controller({'position':position.tolist(),'quaternion':delta.as_quat().tolist()},measured,.02,parameters)
        target=np.asarray(controller.diagnostics()['target_tcp'])
        np.testing.assert_allclose(target[:3,3],expected,atol=1e-12)
        np.testing.assert_allclose(target[:3,:3],rotation@delta.as_matrix()@rotation.T,atol=1e-12)


class FarBranchSolver(Solver):
    """Solver that answers the whole pose in one call, like MMK2's URDF DLS."""
    def solve(self,target):
        result=self.seed.copy();result[:3]=target[:3,3];result[3:]=2.0;return result


def _far_request(controller,parameters,seed):
    controller.reanchor({'position':[0,0,0],'quaternion':[0,0,0,1]},seed,parameters)
    return controller({'position':[1.,0.,0.],'quaternion':[0,0,0,1]},seed,.05,parameters)


def test_max_joint_step_bounds_a_single_tick():
    # MMK2's wrist flipped ~1.5 rad in one tick at q5=0. The binding already
    # declared max_joint_step=0.02, but the generic controller never read it.
    seed=[0.]*6
    out=_far_request(CartesianController(FarBranchSolver()),{'max_joint_step':.02},seed)
    assert np.max(np.abs(np.asarray(out)-seed))<=.02+1e-12


def test_max_joint_velocity_bounds_a_single_tick_by_dt():
    seed=[0.]*6
    out=_far_request(CartesianController(FarBranchSolver()),{'max_joint_velocity':.2},seed)
    assert np.max(np.abs(np.asarray(out)-seed))<=.2*.05+1e-12


def test_both_limits_take_the_tighter_one():
    seed=[0.]*6
    parameters={'max_joint_step':.5,'max_joint_velocity':.2}
    out=_far_request(CartesianController(FarBranchSolver()),parameters,seed)
    assert np.max(np.abs(np.asarray(out)-seed))<=.2*.05+1e-12


def test_without_the_parameters_the_solver_result_is_unchanged():
    seed=[0.]*6
    out=_far_request(CartesianController(FarBranchSolver()),{},seed)
    np.testing.assert_allclose(out,[1.,0.,0.,2.,2.,2.])


def test_joint_step_cap_applies_to_jog_too():
    seed=[0.]*6
    controller=CartesianController(FarBranchSolver(),jog=True)
    parameters={'axes_map':{'x':'x','y':'y'},'linear_speed':.1,'max_joint_step':.02}
    out=controller({'x':1,'y':0},seed,.02,parameters)
    assert np.max(np.abs(np.asarray(out)-seed))<=.02+1e-12


def test_wrappers_that_clip_themselves_opt_out_of_the_shared_cap():
    # FR3/CR5 async controllers already clip each outgoing command against the
    # last one (max_joint_velocity·dt) and let their differential solver bound
    # max_joint_step. Clipping here as well halved the commanded rate, because
    # the inner seed is itself a clipped command one tick old.
    seed=[0.]*6
    parameters={'max_joint_velocity':1.}
    out=_far_request(CartesianController(FarBranchSolver(),step_cap=False),parameters,seed)
    np.testing.assert_allclose(out,[1.,0.,0.,2.,2.,2.])


@pytest.mark.parametrize('bad',[0,-1,float('nan'),'fast'])
def test_invalid_joint_step_limits_are_rejected(bad):
    seed=[0.]*6
    with pytest.raises(ControllerError):
        _far_request(CartesianController(FarBranchSolver()),{'max_joint_step':bad},seed)
    with pytest.raises(ControllerError):
        _far_request(CartesianController(FarBranchSolver()),{'max_joint_velocity':bad},seed)

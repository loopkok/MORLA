"""Actual FR3 geometry: bounded teleop steps, boundary recovery, strict fallback."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
pytest.importorskip('pinocchio')
from morla_fr3_driver.differential_ik import DifferentialIKSolver
from morla_fr3_driver.urdf_ik import IKSolver
from morla_fr3_driver.vr_controller import create_controller, create_jog_controller, validate_parameters

READY=np.array([0.,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0.])


def test_reachable_full_pose_converges_with_coupled_wrist():
    solver=DifferentialIKSolver(); q=READY.copy(); target=solver.fk(q)
    target[:3,:3]=Rotation.from_euler('x',30,degrees=True).as_matrix()@target[:3,:3]
    for tick in range(180):
        # A smooth hand trajectory within the lead cap retains correspondence.
        target[:3,:3]=Rotation.from_euler('x',min(30,(tick+1)/2),degrees=True).as_matrix()@solver.fk(READY)[:3,:3]
        solver.sync_state(q);n=solver.solve(target)
        assert np.max(np.abs(n-q))<=.040000001
        assert solver.last_report['accepted']
        q=n
    actual=solver.fk(q)
    assert np.linalg.norm(actual[:3,3]-target[:3,3])<.0002
    assert Rotation.from_matrix(actual[:3,:3].T@target[:3,:3]).magnitude()<.002
    # Full Jacobian compensates wrist-position coupling, unlike a 3+3 split.
    assert np.linalg.norm(q[:3]-READY[:3])>.01
    assert solver.last_report['converged']


def test_absorbed_boundary_demand_reverses_without_unwinding():
    solver=DifferentialIKSolver();solver.sync_state(READY)
    current=solver.fk(READY);raw=current.copy();raw[0,3]+=5
    solver.solve(raw)
    assert solver.last_report['position_lead_limited']
    before=solver.goal.copy()
    assert np.linalg.norm(before[:3,3]-current[:3,3])<=.050000001
    # Robot has not moved: reverse by 1 cm from a target still 4.99 m away.
    raw[0,3]-=.01;solver.sync_state(READY);solver.solve(raw)
    assert solver.goal[0,3]==pytest.approx(before[0,3]-.01)
    assert not solver.last_report['position_lead_limited']


def test_rotation_absorption_and_no_antipode_chasing():
    solver=DifferentialIKSolver();solver.sync_state(READY);current=solver.fk(READY)
    raw=current.copy();raw[:3,:3]=Rotation.from_euler('x',170,degrees=True).as_matrix()@current[:3,:3]
    q=solver.solve(raw)
    assert solver.last_report['rotation_lead_limited']
    assert np.max(np.abs(q-READY))<=.040000001
    before=solver.goal.copy()
    raw[:3,:3]=Rotation.from_euler('x',-2,degrees=True).as_matrix()@raw[:3,:3]
    solver.sync_state(READY);solver.solve(raw)
    delta=Rotation.from_matrix(solver.goal[:3,:3]@before[:3,:3].T).as_rotvec()
    assert delta[0]==pytest.approx(np.deg2rad(-2))


def test_explicit_seed_is_the_integration_base_and_the_lead_anchor():
    solver=DifferentialIKSolver();solver.sync_state(READY)
    command=READY.copy();command[0]+=np.deg2rad(1.5)
    # At the command's own pose there is nothing left to do: the tick holds the
    # supplied base instead of pulling it back toward the synced measurement.
    np.testing.assert_allclose(solver.solve(solver.fk(command),seed=command),command,atol=1e-12)
    # The absorbing lead window follows the same base.
    hand=solver.fk(command);hand[0,3]+=.5
    solver.solve(hand,seed=command)
    assert solver.last_report['position_lead_limited']
    assert np.linalg.norm(solver.goal[:3,3]-solver.fk(command)[:3,3])<=.050000001


def test_singularity_adaptive_damping_and_static_hold():
    solver=DifferentialIKSolver()
    values=[]
    for q in (READY,np.zeros(6)):
        solver.reset_teleop();solver.sync_state(q);target=solver.fk(q)
        n=solver.solve(target);values.append(solver.last_report['damping'])
        np.testing.assert_allclose(n,q,atol=1e-12)
        assert solver.last_report['converged']
    assert values[1]>values[0]
    assert values[1]==pytest.approx(solver.damping_max,rel=.001)
    target=solver.fk(np.zeros(6));target[0,3]+=.02
    n=solver.solve(target)
    assert np.isfinite(n).all() and np.max(np.abs(n))<=.040000001


def test_unreachable_joints_and_deadline_return_bounded_hold():
    solver=DifferentialIKSolver(joint_lower=(READY-.02).tolist(),joint_upper=(READY+.02).tolist())
    target=solver.fk(READY);target[:3,3]+=[3,2,1];q=READY.copy()
    for _ in range(100):
        solver.sync_state(q);q=solver.solve(target)
        assert np.isfinite(q).all()
        assert np.all(q>=solver.lower_limits) and np.all(q<=solver.upper_limits)
    solver.sync_state(q)
    tick=[0.]
    def expired(): tick[0]+=.1;return tick[0]
    solver.clock=expired
    np.testing.assert_array_equal(solver.solve(target),q)
    assert solver.last_report['reason']=='deadline_hold'
    solver.reset_teleop();assert solver.goal is None and solver.previous_raw is None
    target[0,3]=np.nan
    with pytest.raises(ValueError):solver.solve(target)


@pytest.mark.parametrize('jog',[False,True])
def test_controller_selection_strict_and_differential(jog):
    factory=create_jog_controller if jog else create_controller
    for mode in ('strict','differential'):
        controller=factory({'instance':{'plugin_id':'morla.fairino.fr3','backend':'fake'},
                            'binding':{'parameters':{'ik_mode':mode}}})
        try:
            solver=controller.controller.solver
            assert isinstance(solver,DifferentialIKSolver)==(mode=='differential')
            assert isinstance(solver,IKSolver)
            assert 'solver' in controller.diagnostics()
        finally:controller.close()
    for options in ({'ik_mode':'bogus'},{'damping_min':.2,'damping_max':.1},
                    {'position_lead_m':0},{'rotation_lead_rad':float('nan')},
                    {'command_lead_rad':0},{'command_lead_rad':1.5}):
        with pytest.raises(ValueError):validate_parameters(options,jog=jog)


def test_hardware_mapping_and_reanchor_clear_absorbed_target():
    signs=np.array([-1,1,-1,1,-1,1]);offsets=np.array([.1,-.1,.1,0,.2,0])
    solver=DifferentialIKSolver(joint_signs=signs,joint_offsets=offsets)
    q=(READY-offsets)/signs;target=solver.fk(q)
    for step in range(60):
        requested=target.copy();requested[0,3]+=.005*min(1,(step+1)/20)
        solver.sync_state(q);n=solver.solve(requested)
        assert np.max(np.abs(n-q))<=.040000001;q=n
    assert np.linalg.norm(solver.fk(q)[:3,3]-requested[:3,3])<.0002
    solver.reset_teleop();solver.sync_state(q)
    # New anchor is the actual pose: no residual from the old hand target.
    np.testing.assert_allclose(solver.solve(solver.fk(q)),q,atol=1e-12)


def test_async_diagnostic_sink_correlates_actual_worker_input():
    import time
    controller=create_controller({'instance':{'plugin_id':'morla.fairino.fr3','backend':'fake'},'binding':{'parameters':{}}})
    reports=[];controller.set_diagnostic_sink(reports.append)
    def wait_for(count):
        deadline=time.monotonic()+1
        while len(reports)<count:
            assert time.monotonic()<deadline
            time.sleep(.002)
    try:
        controller.set_trace_context({'source_id':'quest','source_epoch':'epoch','sequence':17,'binding_id':'vr'})
        controller({'position':[0,0,0],'quaternion':[0,0,0,1]},READY,.02,{})
        wait_for(1)
        controller.set_trace_context({'source_id':'quest','source_epoch':'epoch','sequence':18,'binding_id':'vr'})
        controller({'position':[.001,0,0],'quaternion':[0,0,0,1]},READY,.02,{})
        wait_for(2)
        assert reports[-1]['input_context']['sequence']==18
        assert reports[-1]['raw_input']['position']==[.001,0,0]
        assert reports[-1]['filtered_input']['position']==[.001,0,0]
        assert reports[-1]['cartesian']['stage']=='solved'
        assert len(reports[-1]['cartesian']['target_tip'])==4
        assert not reports[-1]['delivered_to_device']
        assert reports[-1]['completed_ns']>=reports[-1]['sample_ns']
    finally:controller.close()


def test_async_reanchor_ack_is_from_the_worker_fk_and_generates_no_ik_motion():
    controller=create_controller({'instance':{'plugin_id':'morla.fairino.fr3','backend':'fake'},
                                  'binding':{'parameters':{}}})
    original=controller.controller.solver.solve
    def forbidden(target):raise AssertionError('reanchor must not solve a motion target')
    controller.controller.solver.solve=forbidden
    try:
        value={'position':[1.,2.,3.],'quaternion':[0.,0.,0.,1.]}
        ack=controller.reanchor(value,READY,{})
        assert ack['acknowledged']
        np.testing.assert_allclose(ack['current_tcp'],controller.controller.solver.fk(READY))
        np.testing.assert_allclose(controller.result[0],READY)
        assert controller.report['cartesian']['reanchor_acknowledged']
        controller.controller.solver.solve=original
        np.testing.assert_allclose(controller(value,READY,.02,{}),READY)
        with pytest.raises(RuntimeError,match='pose'):
            controller.reanchor({'position':[0,0,0],'quaternion':[0,0,0,0]},READY,{})
    finally:controller.close()


def test_fr3_output_obeys_device_speed_and_lead_even_with_larger_binding():
    controller=create_controller({'instance':{'plugin_id':'morla.fairino.fr3','backend':'fake',
        'parameters':{'max_command_lead_rad':.025,'max_velocity_rad_s':.05}},'binding':{'parameters':{}}})
    try:
        # Hold the worker to inject an adversarial but fresh candidate. This
        # tests the output boundary independently of solver convergence.
        with controller.condition:
            controller.result=(READY+1,controller.clock())
            controller.last_output=READY.copy()
            outputs=[]
            for _ in range(50):
                outputs.append(np.array(controller({'position':[0,0,0],'quaternion':[0,0,0,1]},
                    READY,.02,{'command_lead_rad':.8,'max_joint_velocity':.9})))
            values=np.vstack([READY,*outputs])
            assert np.max(np.abs(np.diff(values,axis=0)))<=.00100000001
            assert np.max(np.abs(values-READY))<=.02500000001
    finally:controller.close()

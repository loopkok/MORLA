"""HOME reverses the native path then reaches the final J2=85-degree pose."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nero_robot_control.home_trajectory import home_targets
from nero_robot_control.workpos_trajectory import workpos_targets
from nero_robot_control.waypoint_trajectory import default_waypoints_deg
from nero_robot_control.joint_conventions import mirror_joint_positions


@pytest.mark.parametrize('side',['left','right'])
def test_home_reverses_four_points_then_finishes_at_j2_85(side):
    forward=workpos_targets(side)
    backward=home_targets(side)
    assert len(forward)==4 and len(backward)==5
    for f,b in zip(forward, reversed(backward[:-1])):
        np.testing.assert_array_equal(f,b)
    np.testing.assert_allclose(backward[-1],np.deg2rad([0,85,0,0,0,0,0]))
    backward[0][0]=99
    assert forward[-1][0]!=99
    assert home_targets(side)[0][0]!=99


def test_native_mirror_convention_and_round_trip():
    right=np.array([10.,20.,30.,40.,50.,60.,70.])
    np.testing.assert_array_equal(mirror_joint_positions(right),[-10,20,-30,40,-50,-60,70])
    np.testing.assert_array_equal(mirror_joint_positions(mirror_joint_positions(right)),right)
    for right,left in zip(workpos_targets('right'),workpos_targets('left')):
        np.testing.assert_array_equal(mirror_joint_positions(right),left)


@pytest.mark.parametrize('q',[[0.]*6,[float('nan')]*7])
def test_mirroring_rejects_malformed_joint_targets(q):
    with pytest.raises(ValueError):mirror_joint_positions(q)


def test_deployed_configs_match_four_points_and_endpoint_within_limits():
    yaml=pytest.importorskip('yaml')
    config=Path(__file__).resolve().parents[1]/'config'
    limits=np.array([[-2.705261,2.705261],[-1.745330,1.745330],[-2.757621,2.757621],[-1.012291,2.146755],[-2.757621,2.757621],[-.733039,.959932],[-1.570797,1.570797]])
    for side in ('left','right'):
        params=yaml.safe_load((config/f'nero_driver_{side}.yaml').read_text())[f'/{side}_arm/nero_robot_driver']['ros__parameters']
        points=np.array(params['workpos_waypoints_deg']).reshape(4,7)
        np.testing.assert_array_equal(points,default_waypoints_deg(side))
        np.testing.assert_allclose(params['position_pose'],np.deg2rad(points[-1]))
        assert 'home_j1_deg' not in params and 'home_j2_deg' not in params and 'workpos_j1_deg' not in params
        for q in workpos_targets(side,points)+home_targets(side,points):
            assert np.all(q>=limits[:,0]) and np.all(q<=limits[:,1])

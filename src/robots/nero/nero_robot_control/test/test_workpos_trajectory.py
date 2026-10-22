import numpy as np
import pytest
from nero_robot_control.workpos_trajectory import workpos_targets
from nero_robot_control.waypoint_trajectory import default_waypoints_deg


def test_exact_right_points_with_confirmed_limit_corrections():
    expected=np.array([[0,25,0,123,0,0,-60],[0,25,0,123,-100,42,0],[0,25,0,123,-100,42,-70],[29,31,42,103,-120,24,-10]])
    np.testing.assert_array_equal(default_waypoints_deg('right'),expected)
    np.testing.assert_allclose(workpos_targets('right'),np.deg2rad(expected))


def test_custom_native_points_not_mirrored_twice_or_aliased():
    points=default_waypoints_deg('left')
    original=points.copy()
    targets=workpos_targets('left',points.flatten())
    np.testing.assert_allclose(targets,np.deg2rad(points))
    targets[0][0]=99
    np.testing.assert_array_equal(points,original)
    assert targets[1][0]==0


@pytest.mark.parametrize('points',[[0.]*7,[0.]*27,[0.]*29,[float('nan')]*28,[float('inf')]*28])
def test_invalid_waypoint_config_rejected(points):
    with pytest.raises(ValueError):workpos_targets('left',points)


def test_invalid_side_rejected():
    with pytest.raises(ValueError):workpos_targets('unknown')

"""HOME reverses the working path, then finishes at J2=85 degrees."""
import numpy as np
from .waypoint_trajectory import waypoint_targets

HOME_FINAL_POSE_DEG = (0., 85., 0., 0., 0., 0., 0.)


def home_targets(side, waypoints_deg=None):
    targets = waypoint_targets(side, waypoints_deg, reverse=True)
    # J2/J4/J7 have equal native signs. This final pose is identical on both arms.
    targets.append(np.deg2rad(HOME_FINAL_POSE_DEG))
    return targets

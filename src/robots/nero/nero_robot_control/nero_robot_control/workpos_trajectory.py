"""Working path: all four configured native waypoints in forward order."""
from .waypoint_trajectory import waypoint_targets


def workpos_targets(side, waypoints_deg=None):
    return waypoint_targets(side, waypoints_deg)

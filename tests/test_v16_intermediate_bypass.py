"""v0.16: optional safe bypass of an unreachable INTERMEDIATE waypoint."""
import math
import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.types import Scan, SensorPacket


def packet(c):
    return SensorPacket(0.0, 0.0,
                        Scan(np.full(c.lidar_beams, c.lidar_max_range),
                             np.linspace(-math.pi, math.pi, c.lidar_beams, endpoint=False),
                             c.lidar_max_range), None)


def setup(c, next_path):
    nav = Navigator(c)
    nav.idx = 1
    nav.map.update = lambda pose, scan: None
    attempted = []
    def plan(start, goal):
        attempted.append(tuple(goal))
        return next_path if tuple(goal) == c.waypoints[2] else []
    nav.map.plan = plan
    nav.map.plan_to_goal_region = lambda start, goal, radius: []
    nav.map.segment_clear = lambda start, end, occupied=None: True
    nav.map.obstacles = lambda: np.zeros((nav.map.height, nav.map.width), dtype=bool)
    return nav, attempted


def test_bypass_waits_for_persistent_failure_then_uses_verified_next_intermediate():
    c = Config(nav_intermediate_goal_region_fallback=True,
               nav_bypass_unreachable_intermediate=True,
               nav_bypass_no_route_after_s=0.5)
    nav, attempted = setup(c, [(-0.8,-0.7), (-0.5,-0.5), (0.0,-0.05)])
    pose = np.array([-1.05, -0.84, 0.4])
    cmd, target = nav.tick(0.0, pose, packet(c))
    assert cmd == (0., 0.)
    assert nav.idx == 1 and nav.bypassed_waypoints == []
    cmd, target = nav.tick(0.6, pose, packet(c))
    assert nav.idx == 2
    assert nav.bypassed_waypoints == [1]
    assert nav.last_waypoint_bypassed == 1
    assert nav.last_planner_status == 'waypoint_bypass_path'
    assert tuple(c.waypoints[2]) in attempted
    assert tuple(c.waypoints[3]) not in attempted
    assert target == (-0.8, -0.7) or target == (-0.5, -0.5)
    assert nav.path


def test_disabled_policy_does_not_bypass():
    c = Config(nav_intermediate_goal_region_fallback=True,
               nav_bypass_unreachable_intermediate=False)
    nav, attempted = setup(c, [(-0.8,-0.7), (-0.5,-0.5)])
    pose = np.array([-1.05, -0.84, 0.4])
    cmd, target = nav.tick(12., pose, packet(c))
    assert nav.idx == 1 and nav.bypassed_waypoints == []
    assert tuple(c.waypoints[2]) not in attempted
    assert cmd == (0., 0.)


def test_unreachable_next_waypoint_remains_stationary():
    c = Config(nav_intermediate_goal_region_fallback=True,
               nav_bypass_unreachable_intermediate=True,
               nav_bypass_no_route_after_s=0.)
    nav, attempted = setup(c, [])
    pose = np.array([-1.05, -0.84, 0.4])
    cmd, target = nav.tick(0., pose, packet(c))
    assert tuple(c.waypoints[2]) in attempted
    assert nav.idx == 1 and nav.bypassed_waypoints == []
    assert cmd == (0., 0.)
    assert target == tuple(pose[:2])


def test_never_bypass_to_final_dock():
    c = Config(nav_intermediate_goal_region_fallback=True,
               nav_bypass_unreachable_intermediate=True,
               nav_bypass_no_route_after_s=0.)
    nav, attempted = setup(c, [])
    nav.idx = 2  # last intermediate: only the final docking goal remains
    nav.tick(0.0, np.array([0.25, -0.7, 0.]), packet(c))
    assert tuple(c.waypoints[3]) not in attempted
    assert nav.idx == 2 and nav.bypassed_waypoints == []


def test_actual_grid_disallows_corner_cutting_and_remains_obstacle_aware():
    from turtlebot_integrated.mapping import OccupancyMap
    c = Config()
    m = OccupancyMap(c)
    # This is a synthetic occupied patch around waypoint 1, NOT ground truth
    # provided to the navigator during real operation.
    for iy in range(m.height):
        for ix in range(m.width):
            x,y = m.cell_to_xy((ix,iy))
            if -0.15 <= x <= 0.40 and -0.62 <= y <= 0.01:
                m.log_odds[iy,ix] = 4.0
    start = (-1.05, -0.84)
    current = c.waypoints[1]
    following = c.waypoints[2]
    assert m.plan(start, current) == []
    assert m.plan_to_goal_region(start, current, c.nav_intermediate_goal_radius_m) == []
    next_path = m.plan(start, following)
    assert next_path, 'A* should find a different corridor to next intermediate'
    mask = m.obstacles()
    for a,b in zip(next_path, next_path[1:]):
        assert m.segment_clear(a, b, mask)

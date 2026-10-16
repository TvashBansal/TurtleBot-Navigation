"""v0.11: intermediate region fallback also handles a free but isolated goal cell."""
import math
import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.mapping import OccupancyMap
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.types import Scan, SensorPacket


def _packet(c):
    return SensorPacket(
        0.0, 0.0,
        Scan(np.full(c.lidar_beams, c.lidar_max_range),
             np.linspace(-math.pi, math.pi, c.lidar_beams, endpoint=False),
             c.lidar_max_range),
        None,
    )


def test_goal_region_can_recover_a_free_but_isolated_nominal_goal_cell():
    c = Config()
    om = OccupancyMap(c)
    start, goal = (-0.7, 0.0), (0.0, 0.0)
    gc = om.xy_to_cell(*goal)
    mask = np.zeros((om.height, om.width), dtype=bool)
    # The exact goal cell is FREE, but all of its neighbours are blocked.
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx or dy:
                mask[gc[1] + dy, gc[0] + dx] = True
    om.obstacles = lambda: mask.copy()
    assert not mask[gc[1], gc[0]]
    assert om.plan(start, goal) == []
    alternative = om.plan_to_goal_region(start, goal, 0.24)
    assert alternative, 'Reachable region cell should be considered'
    assert math.dist(alternative[-1], goal) < 0.24
    for xy in alternative:
        cell = om.xy_to_cell(*xy)
        assert not mask[cell[1], cell[0]]


def test_navigator_attempts_region_even_when_exact_goal_cell_is_free():
    c = Config(nav_intermediate_goal_region_fallback=True)
    nav = Navigator(c)
    nav.map.update = lambda pose, scan: None
    nav.map.plan = lambda start, goal: []
    nav.map.plan_to_goal_region = lambda start, goal, radius: [(-1.15, -0.96)]
    nav.map.segment_clear = lambda start, end, occupied=None: True
    empty = np.zeros((nav.map.height, nav.map.width), dtype=bool)
    nav.map.obstacles = lambda: empty.copy()
    pose = np.array([-1.4, -1.1, 0.0])
    _, target = nav.tick(0.0, pose, _packet(c))
    assert nav.last_goal_region_fallback
    assert nav.last_planner_status == 'goal_region_path'
    assert target == (-1.15, -0.96)


def test_disconnected_region_still_holds_safely():
    c = Config(nav_intermediate_goal_region_fallback=True)
    nav = Navigator(c)
    nav.map.update = lambda pose, scan: None
    nav.map.plan = lambda start, goal: []
    nav.map.plan_to_goal_region = lambda start, goal, radius: []
    nav.map.obstacles = lambda: np.zeros((nav.map.height, nav.map.width), dtype=bool)
    pose = np.array([-1.4, -1.1, 0.0])
    (v, w), target = nav.tick(0.0, pose, _packet(c))
    assert (v, w) == (0.0, 0.0)
    assert target == tuple(pose[:2])
    assert nav.last_planner_status == 'no_astar_route'
    assert not nav.last_goal_region_fallback


def test_final_docking_goal_remains_exact_when_feature_enabled():
    c = Config(nav_intermediate_goal_region_fallback=True)
    nav = Navigator(c)
    nav.idx = len(c.waypoints) - 1
    nav.map.update = lambda pose, scan: None
    nav.map.plan = lambda start, goal: []
    def forbidden(*args, **kwargs):
        raise AssertionError('Final docking route must not use the region fallback')
    nav.map.plan_to_goal_region = forbidden
    pose = np.array([0.4, -0.5, 0.0])
    (v, w), target = nav.tick(0.0, pose, _packet(c))
    assert (v, w) == (0.0, 0.0)
    assert target == tuple(pose[:2])

"""v0.10: keep exact final docking goal; permit only safe intermediate goal regions."""
import math
import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.mapping import OccupancyMap
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.types import Scan, SensorPacket


def _open_map():
    c = Config()
    m = OccupancyMap(c)
    return m, np.zeros((m.height, m.width), dtype=bool)


def _packet(cfg):
    return SensorPacket(0.0, 0.0,
                        Scan(np.full(cfg.lidar_beams, 3.2),
                             np.linspace(-math.pi, math.pi, cfg.lidar_beams, endpoint=False), 3.2),
                        None)


def test_region_finds_safe_cell_when_exact_goal_cell_blocked():
    m, mask = _open_map()
    start = (-1.4, 0.)
    goal = (-.2, 0.)
    goal_cell = m.xy_to_cell(*goal)
    mask[goal_cell[1], goal_cell[0]] = True
    m.obstacles = lambda: mask.copy()
    assert m.plan(start, goal) == []
    path = m.plan_to_goal_region(start, goal, .24)
    assert path
    endpoint = path[-1]
    assert math.dist(endpoint, goal) < .24
    for cell_xy in path:
        cell = m.xy_to_cell(*cell_xy)
        assert not mask[cell[1], cell[0]]
    assert m.xy_to_cell(*endpoint) != goal_cell


def test_goal_region_fails_safely_when_all_candidate_cells_blocked():
    m, mask = _open_map()
    start = (-1.4, 0.)
    goal = (-.2, 0.)
    for y in range(m.height):
        for x in range(m.width):
            if math.dist(m.cell_to_xy((x,y)), goal) < .24:
                mask[y,x] = True
    m.obstacles = lambda: mask.copy()
    assert not m.plan_to_goal_region(start, goal, .24)


def test_region_does_not_jump_across_impenetrable_barrier():
    m, mask = _open_map()
    # Full vertical barrier, including boundary: no alternative path.
    column = m.xy_to_cell(0., 0.)[0]
    mask[:, column] = True
    m.obstacles = lambda: mask.copy()
    assert m.plan_to_goal_region((-.9,0.), (.9,0.), .24) == []


def test_goal_region_requires_valid_in_bounds_positions():
    m, _ = _open_map()
    assert m.plan_to_goal_region((-99.,0.),(.9,0.),.24) == []
    assert m.plan_to_goal_region((0.,0.),(99.,0.),.24) == []
    assert m.plan_to_goal_region((0.,0.),(.9,0.),0.) == []


def test_intermediate_goal_region_fallback_is_explicit():
    cfg = Config(nav_local_capture_radius_m=.025, nav_intermediate_goal_region_fallback=True)
    nav = Navigator(cfg)
    nav.map.update=lambda pose,scan: None
    nav.map.plan=lambda start,goal: []
    nav.map.plan_to_goal_region=lambda start,goal,radius: [(-.45,-.88)]
    mask=np.zeros((nav.map.height,nav.map.width), dtype=bool)
    cell=nav.map.xy_to_cell(*cfg.waypoints[0])
    mask[cell[1],cell[0]]=True
    nav.map.obstacles=lambda: mask.copy()
    nav.map.segment_clear=lambda start,end,mask=None: True
    pose = np.array([-1.,-1.,0.])
    _, target = nav.tick(0.0, pose, _packet(cfg))
    assert nav.last_planner_status == 'goal_region_path'
    assert nav.last_goal_region_fallback
    assert target == (-.45,-.88)


def test_final_goal_never_uses_approximation():
    cfg = Config(nav_local_capture_radius_m=.025, nav_intermediate_goal_region_fallback=True)
    nav = Navigator(cfg)
    nav.idx = len(cfg.waypoints) - 1
    nav.map.update=lambda pose,scan: None
    nav.map.plan=lambda start,goal: []
    def forbidden(*args,**kwargs):
        raise AssertionError('Final dock must never use a region fallback')
    nav.map.plan_to_goal_region=forbidden
    pose = np.array([.4,-.5,0.])
    (v,w), target = nav.tick(0.,pose,_packet(cfg))
    assert (v,w) == (0.,0.)
    assert target == (.4,-.5)
    assert nav.last_planner_status == 'no_astar_route'


def test_feature_disabled_keeps_legacy_safe_hold():
    cfg = Config(nav_intermediate_goal_region_fallback=False)
    nav = Navigator(cfg)
    nav.map.update = lambda pose,scan: None
    nav.map.plan = lambda start,goal: []
    mask = np.zeros((nav.map.height,nav.map.width),dtype=bool)
    cell = nav.map.xy_to_cell(*cfg.waypoints[0])
    mask[cell[1],cell[0]] = True
    nav.map.obstacles=lambda: mask.copy()
    def forbidden(*args,**kwargs):
        raise AssertionError('Disabled feature should not be invoked')
    nav.map.plan_to_goal_region = forbidden
    (v,w), target=nav.tick(0.,np.array([-1.,-1.,0.]), _packet(cfg))
    assert (v,w)==(0.,0.)
    assert nav.last_planner_status == 'no_astar_route'

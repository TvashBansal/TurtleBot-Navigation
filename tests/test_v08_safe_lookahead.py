"""v0.8: local targets cannot jump through LiDAR-observed obstacles."""
import math
import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.mapping import OccupancyMap
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.types import Scan, SensorPacket


def test_segment_clear_samples_inflated_occupied_cells():
    c = Config()
    m = OccupancyMap(c)
    occupied = np.zeros((m.height, m.width), dtype=bool)
    cell = m.xy_to_cell(0.0, 0.0)
    occupied[cell[1],cell[0]] = True
    assert not m.segment_clear((-0.7, 0.0),(0.7,0.0),occupied)
    assert m.segment_clear((-0.7,0.6),(0.7,0.6),occupied)
    assert not m.segment_clear((-100.0,0.0),(0,0),occupied)


def test_lookahead_never_shortcuts_through_occupied_cell():
    cfg=Config(nav_lookahead_m=1.3)
    n=Navigator(cfg)
    # Force an ordered A* result which bends above an occupied interior cell.
    n.map.plan = lambda start, goal: [(-.6, 0.0),(-.3,.48),(0.0,.72),(.3,.48),(.6,0.0)]
    mask=np.zeros((n.map.height,n.map.width),dtype=bool)
    blocker=n.map.xy_to_cell(0.0,0.0)
    mask[blocker[1],blocker[0]]=True
    n.map.obstacles = lambda: mask.copy()
    n.map.update = lambda pose,scan: None
    angles=np.linspace(-math.pi,math.pi,cfg.lidar_beams,endpoint=False)
    packet=SensorPacket(0,0,Scan(np.full(cfg.lidar_beams,3.0),angles,3.2),None)
    _,target=n.tick(0.0,np.array([-.85,0.,0.]),packet)
    assert target!=(.6,0.), 'Should not shortcut to far end through obstacle'
    assert n.map.segment_clear((-.85,0.),target)


def test_line_blocked_uses_replan_hold_not_driving_into_obstacle():
    cfg=Config(nav_lookahead_m=.55)
    n=Navigator(cfg)
    n.map.plan=lambda start,goal: [(0.9,0.0)]
    mask=np.ones((n.map.height,n.map.width),dtype=bool)
    n.map.obstacles=lambda:mask.copy()
    n.map.update=lambda pose,scan:None
    angles=np.linspace(-math.pi,math.pi,cfg.lidar_beams,endpoint=False)
    packet=SensorPacket(0,0,Scan(np.full(cfg.lidar_beams,3.0),angles,3.2),None)
    # Use estimated pose well inside the global grid.
    (v,w), target=n.tick(0.0,np.array([0.,0.,0.]),packet)
    assert v==0.0
    assert target==(0.,0.)
    assert n.path==[]

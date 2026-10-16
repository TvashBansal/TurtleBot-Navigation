"""Safe v0.9 local-target behavior and explicit planner-state diagnostics."""
import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.control import LQRTracker
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.types import Scan, SensorPacket


def _packet(cfg):
    return SensorPacket(0., 0., Scan(np.full(cfg.lidar_beams, 3.0),
          np.linspace(-np.pi, np.pi, cfg.lidar_beams, endpoint=False), 3.2), None)


def test_default_deadband_is_unchanged():
    cfg=Config()
    assert cfg.nav_local_capture_radius_m == 0.08
    assert LQRTracker(cfg).track(np.array([0.,0.,0.]),(.055,0.)) == (0.,0.)


def test_candidate_tracker_can_advance_to_a_close_safe_grid_node():
    cfg=Config(nav_local_capture_radius_m=0.025)
    v,w=LQRTracker(cfg).track(np.array([0.,0.,0.]),(.055,0.))
    assert v > 0 and np.isfinite(w)


def test_no_astar_route_still_holds_safe():
    cfg=Config(nav_local_capture_radius_m=0.025)
    nav=Navigator(cfg)
    nav.map.update=lambda pose,scan: None
    nav.map.plan=lambda start,goal: []
    (v,w),target=nav.tick(0.,np.array([0.,0.,0.]),_packet(cfg))
    assert nav.last_planner_status == 'no_astar_route'
    assert (v,w)==(0.,0.)
    assert target==(0.,0.)


def test_visible_short_path_node_is_not_automatically_discarded():
    cfg=Config(nav_local_capture_radius_m=0.025)
    nav=Navigator(cfg)
    nav.map.update=lambda pose,scan: None
    nav.map.plan=lambda start,goal: [(0.055,0.)]
    nav.map.segment_clear=lambda start,end,mask=None: True
    (v,w),target=nav.tick(0.,np.array([0.,0.,0.]),_packet(cfg))
    assert nav.last_planner_status=='path_following'
    assert np.isclose(nav.last_target_distance_m,.055)
    assert np.isclose(target[0],.055)
    assert v>0


def test_unseen_path_remains_stationary_even_with_smaller_deadband():
    cfg=Config(nav_local_capture_radius_m=0.025)
    nav=Navigator(cfg)
    nav.map.update=lambda pose,scan: None
    nav.map.plan=lambda start,goal: [(0.20,0.)]
    nav.map.segment_clear=lambda start,end,mask=None: False
    (v,w),target=nav.tick(0.,np.array([0.,0.,0.]),_packet(cfg))
    assert nav.last_planner_status=='no_visible_onward_path'
    assert (v,w)==(0.,0.)
    assert target==(0.,0.)
    assert nav.path==[]

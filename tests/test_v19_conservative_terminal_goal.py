"""v0.19: conservative estimated-state stop, no truth or score shortcuts."""
import math

import numpy as np
import pytest

from turtlebot_integrated.config import Config
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.types import Scan, SensorPacket


def packet(c):
    return SensorPacket(
        0.0, 0.0,
        Scan(np.full(c.lidar_beams, c.lidar_max_range),
             np.linspace(-math.pi, math.pi, c.lidar_beams, endpoint=False),
             c.lidar_max_range),
        None,
    )


def terminal_nav(c):
    nav = Navigator(c)
    nav.idx = len(c.waypoints) - 1
    return nav


def test_old_default_stop_policy_preserved():
    c = Config()
    nav = terminal_nav(c)
    v, _ = nav.tick(0.0, np.array([c.dock_pose[0] - 0.17, 0., 0.17]), packet(c))
    assert nav.done and v == (0.0, 0.0)
    assert nav.last_stop_reason == 'goal_reached'


def test_position_margin_prevents_premature_goal_reached():
    c = Config(nav_dock_position_margin_m=0.04,
               nav_dock_heading_margin_rad=0.035)
    nav = terminal_nav(c)
    nav.tick(0.0, np.array([c.dock_pose[0] - 0.17, 0., 0.10]), packet(c))
    assert not nav.done
    assert nav.last_stop_reason != 'goal_reached'


def test_heading_margin_prevents_premature_goal_reached():
    c = Config(nav_dock_position_margin_m=0.04,
               nav_dock_heading_margin_rad=0.035)
    nav = terminal_nav(c)
    nav.tick(0.0, np.array([c.dock_pose[0] - 0.12, 0., 0.16]), packet(c))
    assert not nav.done
    assert nav.last_stop_reason != 'goal_reached'


def test_stricter_estimated_goal_still_reachable_with_original_lqr():
    c = Config(nav_dock_position_margin_m=0.04,
               nav_dock_heading_margin_rad=0.035)
    nav = terminal_nav(c)
    cmd, target = nav.tick(0.0, np.array([c.dock_pose[0] - 0.135, 0., 0.14]), packet(c))
    assert nav.done and cmd == (0.0, 0.0)
    assert target == tuple(c.dock_pose[:2])


def test_score_tolerance_remains_separate_and_unchanged():
    c = Config.load('configs/mujoco_v19_precision_trial.yaml')
    assert c.dock_tolerance == pytest.approx(0.18)
    assert c.nav_dock_position_margin_m == pytest.approx(0.04)
    assert c.nav_dock_heading_margin_rad == pytest.approx(0.035)
    assert c.nav_local_capture_radius_m == pytest.approx(0.025)


@pytest.mark.parametrize('pos,heading', [(-.001,0), (.18,0), (0,-.001), (0,.18)])
def test_reject_invalid_margins(pos, heading):
    c = Config(nav_dock_position_margin_m=pos, nav_dock_heading_margin_rad=heading)
    with pytest.raises(ValueError):
        Navigator(c)

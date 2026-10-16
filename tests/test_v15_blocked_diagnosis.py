"""Offline blocked-scenario diagnostics do not alter navigation behavior."""
import json
from pathlib import Path

from turtlebot_integrated.config import Config
from turtlebot_integrated.scenarios import get_scenario
from turtlebot_integrated.geometry import point_rect_distance


def test_blocked_extra_obstacle_near_second_waypoint():
    cfg = Config()
    blocked = get_scenario('blocked')
    nominal = get_scenario('nominal')
    extra = blocked.obstacles[len(nominal.obstacles):]
    assert len(extra) == 1
    distance = point_rect_distance(*cfg.waypoints[1], extra[0])
    assert 0 < distance < cfg.map_inflation


def test_scoring_margin_documented_by_source():
    cfg = Config()
    assert cfg.dock_tolerance == 0.18
    assert cfg.dock_tolerance + 0.06 == 0.24

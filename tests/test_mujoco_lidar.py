"""MuJoCo smoke tests for LiDAR self-intersection and clear-ahead sensing."""
from pathlib import Path

import numpy as np
import pytest

from turtlebot_integrated.config import Config
from turtlebot_integrated.mujoco_backend import scene_xml
from turtlebot_integrated.scenarios import get_scenario


def test_lidar_origin_is_clear_of_chassis_in_scene():
    xml = scene_xml(Config(), get_scenario("nominal", 7), Path("marker.png"))
    assert xml.count('pos="0 0 0.105"') == Config().lidar_beams


def test_mujoco_front_lidar_does_not_see_its_own_bumper():
    pytest.importorskip("mujoco")
    from turtlebot_integrated.mujoco_backend import MuJoCoBackend
    cfg = Config(lidar_noise=0)
    sim = MuJoCoBackend(cfg, get_scenario("nominal", 7), seed=7, record_camera=False)
    try:
        packet = sim.observe()
        distances = packet.scan.ranges[np.abs(packet.scan.angles) < 0.4]
        # The nominal starting pose has no external obstacle within 0.30 m
        # of the LiDAR's forward ray cone; lower values suggest self-hits.
        assert float(distances.min()) > 0.30, distances.tolist()
    finally:
        sim.close()

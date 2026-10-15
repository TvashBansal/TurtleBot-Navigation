"""The encoder calibration is allowed to differ from motor/physics geometry."""
import numpy as np
import pytest
from turtlebot_integrated.config import Config
from turtlebot_integrated.ekf import PoseEKF


def test_default_keeps_original_prediction():
    c = Config()
    e = PoseEKF(c, np.zeros(3))
    e.predict(2.0, 2.0)
    assert e.x[0] == pytest.approx(2*c.wheel_radius)


def test_radius_adjusts_odom_only():
    c = Config(odom_wheel_radius=0.0356, odom_wheel_separation=0.198)
    e = PoseEKF(c, np.zeros(3))
    e.predict(2.0, 2.0)
    assert e.x[0] == pytest.approx(2*c.odom_wheel_radius)
    assert c.wheel_radius == pytest.approx(.033)
    assert c.wheel_separation == pytest.approx(.16)


def test_separation_adjusts_turn_without_ground_truth():
    c = Config(odom_wheel_radius=0.0356, odom_wheel_separation=0.198)
    e = PoseEKF(c, np.zeros(3))
    e.predict(-1.0, 1.0)
    assert e.x[2] == pytest.approx(2*.0356/.198)


def test_nonpositive_odom_is_rejected():
    c = Config(odom_wheel_separation=-0.1)
    with pytest.raises(ValueError):
        PoseEKF(c, np.zeros(3)).predict(0.0, 0.1)


def test_config_file_roundtrip():
    c = Config.load('configs/mujoco_odom_candidate.yaml')
    assert c.odom_wheel_separation == pytest.approx(.198)
    assert c.odom_wheel_radius == pytest.approx(.0356)

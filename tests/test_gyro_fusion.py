"""Regression checks for optional IMU yaw-rate fusion; no ground truth in estimator."""
import math
import numpy as np
import pytest
from turtlebot_integrated.config import Config
from turtlebot_integrated.ekf import PoseEKF
from turtlebot_integrated.hardware import SensorBridge, SafetyGate


def test_gyro_off_preserves_legacy_odom_even_if_measurement_available():
    cfg=Config(use_gyro_yaw=False)
    baseline=PoseEKF(cfg,np.zeros(3))
    candidate=PoseEKF(cfg,np.zeros(3))
    baseline.predict(-2.,2.)
    candidate.predict(-2.,2.,gyro_delta_yaw=.2)
    assert np.allclose(baseline.x,candidate.x)
    assert candidate.last_used_gyro is False


def test_gyro_pulls_heading_toward_measured_increment():
    cfg=Config(use_gyro_yaw=True,gyro_yaw_weight=.98)
    ekf=PoseEKF(cfg,np.zeros(3))
    ekf.predict(-2.,2.,gyro_delta_yaw=.42)
    encoder_only=cfg.wheel_radius*4/cfg.wheel_separation
    assert abs(ekf.x[2]-.42)<abs(encoder_only-.42)
    assert ekf.last_used_gyro
    assert math.isfinite(ekf.last_odom_minus_gyro)
    assert np.all(np.linalg.eigvalsh(ekf.P)>=-1e-10)


def test_nonfinite_or_missing_gyro_falls_back_to_wheels():
    cfg=Config(use_gyro_yaw=True)
    for bad in (None,float('nan'),float('inf')):
        ekf=PoseEKF(cfg,np.zeros(3))
        ekf.predict(-1.,1.,gyro_delta_yaw=bad)
        assert not ekf.last_used_gyro
        assert math.isfinite(ekf.x[2])
        assert ekf.x[2]==pytest.approx(2*cfg.wheel_radius/cfg.wheel_separation)


def test_weight_validation():
    ekf=PoseEKF(Config(use_gyro_yaw=True,gyro_yaw_weight=1.5),np.zeros(3))
    with pytest.raises(ValueError,match='gyro_yaw_weight'):
        ekf.predict(0.,0.,gyro_delta_yaw=.1)


def test_hardware_imu_staleness_and_motion_gate():
    cfg=Config(use_gyro_yaw=True)
    bridge=SensorBridge(cfg)
    bridge.encoder_update(0.,0.,1.)
    bridge.scan_update([1.5,1.5],-.5,.5,3.,1.)
    bridge.imu_update(.35,1.)
    packet=bridge.packet()
    assert packet.gyro_z_rad_s==pytest.approx(.35)
    status=bridge.status(1.1)
    assert status.imu_ready
    gate=SafetyGate(cfg)
    assert gate.apply(.1,.2,status,np.array([0.,0.,0.]),True,True)[0]>.0
    bridge.encoder_update(0.,0.,1.5)
    bridge.scan_update([1.5,1.5],-.5,.5,3.,1.5)
    assert bridge.packet().gyro_z_rad_s is None
    status=bridge.status(1.5)
    assert not status.imu_ready
    assert gate.apply(.1,.2,status,np.array([0.,0.,0.]),True,True)[:2]==(0.,0.)


def test_simulator_mjcf_has_real_gyro_sensor():
    from pathlib import Path
    from turtlebot_integrated.mujoco_backend import scene_xml
    from turtlebot_integrated.scenarios import get_scenario
    xml=scene_xml(Config(),get_scenario('nominal'),Path('/tmp/marker.png'))
    assert 'name="imu_gyro" site="imu_site"' in xml
    assert 'name="imu_site"' in xml

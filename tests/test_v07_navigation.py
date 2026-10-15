import math
import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.control import LQRTracker
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.types import Scan, SensorPacket


def test_default_controller_remains_unchanged():
    c=Config()
    v,w=LQRTracker(c).track(np.array([0.,0.,0.]), (math.cos(.3),math.sin(.3)))
    assert np.isclose(v,c.max_speed)
    assert np.isclose(w,c.max_omega)


def test_scaled_lqr_less_saturated_and_keeps_forward_progress():
    c=Config(lqr_steering_scale=.20,lqr_speed_limit_m_s=.15,
             lqr_heading_speed_scheduling=True,
             lqr_forward_heading_cutoff_rad=1.25)
    v,w=LQRTracker(c).track(np.array([0.,0.,0.]),(math.cos(.3),math.sin(.3)))
    assert 0 < v <= .15
    assert 0 < w < .7


def test_turn_in_place_for_large_heading_error():
    c=Config(lqr_forward_heading_cutoff_rad=1.25,lqr_turn_kp=1.2)
    v,w=LQRTracker(c).track(np.array([0.,0.,0.]), (-1.,0.))
    assert v==0 and abs(w)>0


def test_scan_brake_is_not_disabled_and_stop_reason_is_logged():
    c=Config(lqr_steering_scale=.2)
    n=Navigator(c)
    scan=Scan(np.full(c.lidar_beams,1.0),np.linspace(-math.pi,math.pi,c.lidar_beams,endpoint=False),c.lidar_max_range)
    # A simulated LiDAR front hit must stop the robot even with new LQR settings.
    scan.ranges[len(scan.ranges)//2]=.16
    packet=SensorPacket(0.,0.,scan,None)
    (v,w),_ = n.tick(0.0,np.array(c.start_pose),packet)
    assert v==0
    assert n.last_stop_reason=='safety_brake'
    assert n.last_front_clearance_m < .26


def test_v07_settings_load_from_yaml():
    c=Config.load('configs/mujoco_v07_steering.yaml')
    assert c.lqr_steering_scale==.2
    assert c.nav_lookahead_m==.55

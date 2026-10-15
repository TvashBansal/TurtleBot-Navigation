from turtlebot_integrated.config import Config
from turtlebot_integrated.ekf import PoseEKF
from turtlebot_integrated.navigation import Navigator
from turtlebot_integrated.kinematic import KinematicTestBackend
from turtlebot_integrated.scenarios import get_scenario
import numpy as np

def test_nominal_kinematic_closed_loop_reaches_dock_without_truth_in_controller():
    cfg=Config(max_duration=35)
    sim=KinematicTestBackend(cfg,get_scenario('nominal'),seed=7)
    ekf=PoseEKF(cfg,np.array(cfg.start_pose))
    nav=Navigator(cfg)
    prev=None
    for _ in range(int(cfg.max_duration/cfg.dt)):
        packet=sim.observe()
        if prev is not None:ekf.predict(packet.encoder_left-prev[0],packet.encoder_right-prev[1])
        prev=(packet.encoder_left,packet.encoder_right)
        ekf.update_camera(packet.camera,cfg.marker_xy)
        cmd,_=nav.tick(sim.time,ekf.x,packet)
        sim.step(*cmd)
        if nav.done:break
    assert nav.done
    assert not sim.collision
    assert np.linalg.norm(sim.get_truth()[:2]-np.array(cfg.dock_pose[:2]))<.2

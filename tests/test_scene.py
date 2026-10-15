from pathlib import Path
from turtlebot_integrated.config import Config
from turtlebot_integrated.scenarios import get_scenario
from turtlebot_integrated.mujoco_backend import scene_xml

def test_mujoco_scene_has_motor_joint_camera_rangefinders():
    cfg=Config(lidar_beams=16)
    xml=scene_xml(cfg,get_scenario('nominal'),Path('/tmp/aruco_0.png'))
    assert xml.count('<rangefinder')==cfg.lidar_beams
    assert '<freejoint' in xml and '<camera name="front_camera"' in xml
    assert '<velocity name="motor_left"' in xml
    assert '<velocity name="motor_right"' in xml
    assert '<jointpos name="enc_left"' in xml

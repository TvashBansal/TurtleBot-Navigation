"""Regression checks ensuring the original MuJoCo model stays the default."""
from pathlib import Path
from turtlebot_integrated.config import Config
from turtlebot_integrated.mujoco_backend import scene_xml
from turtlebot_integrated.scenarios import get_scenario


def test_default_scene_has_no_new_support():
    c=Config()
    xml=scene_xml(c,get_scenario('nominal'),Path('marker.png'))
    assert not c.mujoco_front_support_enabled
    assert 'name="front_support"' not in xml
    assert 'name="rear_castor"' in xml


def test_opt_in_scene_adds_passive_front_contact_but_no_actuation():
    c=Config(mujoco_front_support_enabled=True)
    xml=scene_xml(c,get_scenario('nominal'),Path('marker.png'))
    assert xml.count('name="front_support"') == 1
    assert 'pos="0.105 0 -0.065"' in xml
    assert 'friction="0.03 0.002 0.0001"' in xml
    assert xml.count('<velocity name="motor_left"') == 1
    assert xml.count('<velocity name="motor_right"') == 1
    assert xml.count('<rangefinder') == c.lidar_beams
    assert 'name="imu_gyro"' in xml


def test_yaml_enables_support_only_for_candidate():
    from pathlib import Path
    path=Path(__file__).resolve().parents[1]/'configs'/'mujoco_v12_front_support_probe.yaml'
    c=Config.load(path)
    assert c.mujoco_front_support_enabled
    assert c.max_duration == 30.0

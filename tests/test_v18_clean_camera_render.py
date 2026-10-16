"""Verify camera debug-ray suppression without altering actual LiDAR values."""
import numpy as np
from turtlebot_integrated.mujoco_backend import camera_scene_options, MuJoCoBackend


class FakeVisualOption:
    def __init__(self):
        self.flags = [None] * 12


class FakeMuJoCo:
    class mjtVisFlag:
        mjVIS_RANGEFINDER = 8
    MjvOption = FakeVisualOption

    @staticmethod
    def mjv_defaultOption(opts):
        opts.flags[:] = [1] * 12


def test_camera_disables_only_rangefinder_visuals():
    opt = camera_scene_options(FakeMuJoCo)
    assert opt.flags[FakeMuJoCo.mjtVisFlag.mjVIS_RANGEFINDER] == 0
    assert all(v == 1 for i, v in enumerate(opt.flags)
               if i != FakeMuJoCo.mjtVisFlag.mjVIS_RANGEFINDER)


def test_camera_renderer_uses_clean_visual_options():
    class Recorder:
        def __init__(self):
            self.args = None
        def update_scene(self, *args, **kwargs):
            self.args = (args, kwargs)
        def render(self):
            return np.ones((2, 3, 3), np.uint8)
    sim = MuJoCoBackend.__new__(MuJoCoBackend)
    sim.renderer = Recorder()
    sim.data = object()
    sim.camera_scene_option = camera_scene_options(FakeMuJoCo)
    rgb = sim.render_rgb()
    assert rgb.shape == (2, 3, 3)
    assert sim.renderer.args[1]['camera'] == 'front_camera'
    assert sim.renderer.args[1]['scene_option'] is sim.camera_scene_option


def test_disabled_renderer_returns_no_frame():
    sim = MuJoCoBackend.__new__(MuJoCoBackend)
    sim.renderer = None
    assert sim.render_rgb() is None

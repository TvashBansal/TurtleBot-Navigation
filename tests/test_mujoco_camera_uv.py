"""Checks the corrected MJCF texture and the real rendered sensor path."""
from pathlib import Path
import cv2
import numpy as np
import pytest
from turtlebot_integrated.camera import make_marker, marker_from_rgb
from turtlebot_integrated.config import Config
from turtlebot_integrated.mujoco_backend import make_mujoco_uv_marker
from turtlebot_integrated.scenarios import get_scenario


def test_mujoco_texture_corrects_horizontal_reflection_without_overwriting_source(tmp_path:Path):
    original=tmp_path/'original.png'
    corrected=tmp_path/'corrected.png'
    make_marker(str(original))
    original_pixels=cv2.imread(str(original),cv2.IMREAD_GRAYSCALE)
    make_mujoco_uv_marker(original,corrected)
    corrected_pixels=cv2.imread(str(corrected),cv2.IMREAD_GRAYSCALE)
    assert np.array_equal(corrected_pixels, original_pixels[:,::-1])
    assert np.array_equal(cv2.imread(str(original),cv2.IMREAD_GRAYSCALE), original_pixels)


def test_mujoco_rendered_camera_sees_correct_marker_without_image_flipping():
    pytest.importorskip('mujoco')
    from turtlebot_integrated.mujoco_backend import MuJoCoBackend
    cfg=Config()
    sim=MuJoCoBackend(cfg,get_scenario('nominal',7),seed=7)
    try:
        sim.data.qpos[:7]=[0.50,0.0,0.088,1,0,0,0]
        sim.mj.mj_forward(sim.model,sim.data)
        rgb=sim.render_rgb('front_camera')
        measured=marker_from_rgb(rgb,cfg)
        assert measured is not None, 'Rendered marker still undetectable without frame reflection'
        assert abs(measured.bearing)<0.18, measured.bearing
        assert abs(measured.distance-(cfg.marker_xy[0]-.50))<0.25, measured.distance
    finally:
        sim.close()

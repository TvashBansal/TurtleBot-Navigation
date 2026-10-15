"""Deterministic camera regression checks; no MuJoCo needed."""
import cv2
import numpy as np
import pytest
from turtlebot_integrated.camera import make_marker, marker_from_rgb
from turtlebot_integrated.config import Config


def test_multiscale_preserves_standard_detected_measurements(tmp_path):
    marker=tmp_path/'marker.png'
    make_marker(str(marker))
    image=cv2.imread(str(marker),cv2.IMREAD_COLOR)
    rgb=np.full((480,640,3),255,dtype=np.uint8)
    scaled=cv2.resize(image,(104,104),interpolation=cv2.INTER_NEAREST)
    rgb[170:274,268:372]=cv2.cvtColor(scaled,cv2.COLOR_BGR2RGB)
    standard=marker_from_rgb(rgb,Config(camera_detection_mode='standard'))
    multi=marker_from_rgb(rgb,Config(camera_detection_mode='multiscale'))
    assert standard is not None and multi is not None
    assert standard.distance==pytest.approx(multi.distance,abs=1e-8)
    assert standard.bearing==pytest.approx(multi.bearing,abs=1e-8)


def test_multiscale_never_fabricates_marker_on_blank_frame():
    rgb=np.full((480,640,3),127,dtype=np.uint8)
    assert marker_from_rgb(rgb,Config(camera_detection_mode='multiscale')) is None


def test_config_mode_validation():
    rgb=np.zeros((480,640,3),dtype=np.uint8)
    with pytest.raises(ValueError, match='camera_detection_mode'):
        marker_from_rgb(rgb,Config(camera_detection_mode='typo'))


def test_multiscale_corners_are_in_original_coordinates(tmp_path):
    marker=tmp_path/'marker.png'
    make_marker(str(marker))
    image=cv2.imread(str(marker),cv2.IMREAD_COLOR)
    rgb=np.full((480,640,3),255,dtype=np.uint8)
    scaled=cv2.resize(image,(90,90),interpolation=cv2.INTER_AREA)
    rgb[180:270,280:370]=cv2.cvtColor(scaled,cv2.COLOR_BGR2RGB)
    detection=marker_from_rgb(rgb,Config(camera_detection_mode='multiscale'))
    assert detection is not None
    assert np.all(detection.corners[:,0] > 200)
    assert np.all(detection.corners[:,0] < 440)
    assert np.all(detection.corners[:,1] > 120)
    assert np.all(detection.corners[:,1] < 330)


def test_multiscale_recovers_very_small_tag_corners(tmp_path):
    from turtlebot_integrated.camera import _detect_marker_corners
    marker=tmp_path/'marker.png'
    make_marker(str(marker))
    raw=cv2.imread(str(marker),cv2.IMREAD_GRAYSCALE)
    frame=np.full((480,640),255,dtype=np.uint8)
    frame[230:243,320:333]=cv2.resize(raw,(13,13),interpolation=cv2.INTER_AREA)
    # This tests true corner recognition. At this simulated pixel size, PnP
    # estimates the tag outside our configured reliable detection range.
    corners=_detect_marker_corners(frame,0,'multiscale')
    assert corners is not None
    assert np.all(corners[:,0]>=319) and np.all(corners[:,0]<=334)

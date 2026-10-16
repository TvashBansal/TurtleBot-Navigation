"""Unit tests for OFFLINE-only visibility estimation, no MuJoCo dependency."""
import importlib.util
import math
from pathlib import Path
from turtlebot_integrated.config import Config

MODULE = Path(__file__).resolve().parents[1] / 'scripts' / 'diagnose_camera_visibility_v17.py'
spec = importlib.util.spec_from_file_location('diagnose_camera_visibility_v17', MODULE)
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


def test_segment_rectangle_intersection_and_clearance():
    r=(0.25,0.6,-0.7,-0.34)
    assert diag.segment_intersects_rect((0,-0.5),(1,-0.5),r)
    assert not diag.segment_intersects_rect((0,0),(1,0),r)
    assert not diag.segment_intersects_rect((0,-1),(1,-1),r)


def test_forward_marker_is_visible_when_not_occluded():
    cfg=Config()
    p=diag.visibility_at_pose([1.05,0.0,0.0],cfg,[])
    assert p['opportunity']
    assert abs(p['bearing_deg']) < .01


def test_heading_away_from_marker_is_out_of_view():
    cfg=Config()
    p=diag.visibility_at_pose([1.05,0,math.pi],cfg,[])
    assert not p['opportunity']


def test_los_occlusion_prevents_opportunity():
    cfg=Config()
    p=diag.visibility_at_pose([0.0,-0.5,0.0],cfg,[(0.3,0.6,-0.7,0.2)])
    assert p['occluded_obstacle_indices']==[0]
    assert not p['opportunity']


def test_spacing_of_reconstruction_samples():
    assert diag.evenly_spaced_indices([],8)==[]
    assert diag.evenly_spaced_indices(list(range(80)),1)==[79]
    assert diag.evenly_spaced_indices(list(range(80)),8)[-1]==79
    assert len(diag.evenly_spaced_indices([1,3,5],8))==3

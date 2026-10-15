import numpy as np
import cv2
from pathlib import Path
from turtlebot_integrated.camera import marker_from_rgb, make_marker
from turtlebot_integrated.config import Config

def test_marker_detector_standalone_render(tmp_path:Path):
    p=tmp_path/'marker.png'
    make_marker(str(p))
    gray=cv2.resize(cv2.imread(str(p),cv2.IMREAD_GRAYSCALE),(220,220),interpolation=cv2.INTER_NEAREST)
    image=np.full((480,640,3),255,np.uint8)
    image[130:350,210:430]=cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)
    det=marker_from_rgb(image,Config())
    assert det is not None
    assert det.distance>0.15
    assert abs(det.bearing)<.1

def test_marker_missing_returns_none():
    image=np.full((480,640,3),255,np.uint8)
    assert marker_from_rgb(image,Config()) is None

import math
import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.ekf import PoseEKF
from turtlebot_integrated.types import CameraDetection
from turtlebot_integrated.geometry import landmark_prediction,wrap

def test_predict_forward_and_turn():
    c=Config();e=PoseEKF(c,np.array([0.,0.,0.]))
    e.predict(1.0,1.0)
    assert np.isclose(e.x[0],c.wheel_radius)
    assert abs(e.x[1])<1e-9
    e.predict(-1.,1.)
    assert e.x[2]>0

def test_camera_update_corrects_biased_pose():
    c=Config()
    e=PoseEKF(c,np.array([.12,-.08,.06]))
    marker=c.marker_xy
    z,_=landmark_prediction(np.array([0.,0.,0.]),marker)
    before=np.linalg.norm(e.x)
    assert e.update_camera(CameraDetection(*z),marker)
    assert np.linalg.norm(e.x)<before
    assert np.all(np.linalg.eigvalsh(e.P)>=0)

def test_bearing_wrap():
    assert abs(wrap(3*math.pi)-(-math.pi))<1e-12

def test_camera_outlier_rejection():
    e=PoseEKF(Config(),np.array([0.,0.,0.]))
    p=e.x.copy()
    assert not e.update_camera(CameraDetection(200,1.5),(1.8,0.))
    assert np.array_equal(e.x,p)

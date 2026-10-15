import numpy as np
from turtlebot_integrated.geometry import landmark_prediction

def test_local_controllability_when_moving():
    v=.16
    A=np.array([[0,0,0],[0,0,v],[0,0,0]],float)
    B=np.array([[-1,0],[0,0],[0,-1]],float)
    Co=np.column_stack([B,A@B,A@A@B])
    assert np.linalg.matrix_rank(Co)==3

def test_bearing_range_observability_requires_motion():
    _,H=landmark_prediction(np.array([0.,0.,0.]),(1.82,0.))
    A=np.array([[0,0,0],[0,0,.16],[0,0,0]],float)
    assert np.linalg.matrix_rank(H)==2
    O=np.vstack([H,H@A,H@A@A])
    assert np.linalg.matrix_rank(O)==3
    assert np.linalg.matrix_rank(np.vstack([H,H@np.zeros((3,3))]))==2

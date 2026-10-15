import math
import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.control import LQRTracker
from turtlebot_integrated.mapping import OccupancyMap
from turtlebot_integrated.types import Scan

def test_lqr_forward_command():
    v,w=LQRTracker(Config()).track(np.array([0.,0.,0.]),(1.,0.))
    assert v>0
    assert abs(w)<1e-6

def test_lqr_turns_to_side_target():
    v,w=LQRTracker(Config()).track(np.array([0.,0.,0.]),(0.,1.))
    assert v==0 and w>0

def test_lidar_map_and_plan():
    c=Config();m=OccupancyMap(c)
    d=np.ones(72)*c.lidar_max_range;d[36]=0.4
    angles=np.linspace(-math.pi,math.pi,72,endpoint=False)
    pose=np.array([0.,0.,0.])
    for _ in range(4):m.update(pose,Scan(d,angles,c.lidar_max_range))
    assert m.log_odds[m.xy_to_cell(.4,0.)[1],m.xy_to_cell(.4,0.)[0]]>0
    path=m.plan((0,0),(1,0))
    assert path

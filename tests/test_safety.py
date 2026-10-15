import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.hardware import SensorBridge,SafetyGate

def test_missing_sensor_prevents_motion():
    c=Config();g=SafetyGate(c);b=SensorBridge(c)
    s=b.status(3.)
    assert g.apply(.2,2,s,np.array([0.,0.,0.]),False,True)[0:2]==(0,0)

def test_dry_run_and_velocity_clamps():
    c=Config();g=SafetyGate(c);b=SensorBridge(c)
    b.encoder_update(0,0,3.);b.scan_update([1.,1.],-.5,.5,3.,3.)
    s=b.status(3.1)
    assert g.apply(.2,2,s,np.array([0.,0.,0.]),True,False)[:2]==(0,0)
    v,w,_=g.apply(.2,2,s,np.array([0.,0.,0.]),True,True)
    assert v==.12 and w==.8

def test_consumes_camera_only_once():
    from turtlebot_integrated.types import CameraDetection
    b=SensorBridge(Config());b.encoder_update(0,0,1);b.scan_update([1.,2.],0.,.2,3.,1)
    b.camera_update(CameraDetection(1.,.1),1.)
    assert b.packet().camera is not None
    assert b.packet().camera is None

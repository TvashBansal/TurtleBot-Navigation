import numpy as np
from turtlebot_integrated.video import diagnostic_frame
from turtlebot_integrated.types import StepLog

def test_video_composition():
    frame=np.full((480,640,3),128,dtype=np.uint8)
    log=StepLog(1.,[1,2,0],[1.1,2.1,.03],[.02,.03,.04],[.1,.2],[1.5,0],True,.2,False)
    out=diagnostic_frame(frame,frame,log)
    assert out.shape==(660,1280,3)
    assert out.dtype==np.uint8

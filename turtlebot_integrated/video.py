"""Create synchronized overhead/camera/diagnostic frames from MuJoCo-only imagery."""
import math
import numpy as np
import cv2
from .types import StepLog

def diagnostic_frame(overhead_rgb:np.ndarray, front_rgb:np.ndarray,log:StepLog):
    width,height=640,480
    def panel(im):
        return cv2.resize(np.asarray(im,dtype=np.uint8),(width,height))
    top=panel(overhead_rgb)
    camera=panel(front_rgb)
    cv2.rectangle(top,(0,0),(width,40),(14,22,33),-1)
    cv2.putText(top,'MuJoCo: overhead (evaluation only)',(10,26),cv2.FONT_HERSHEY_SIMPLEX,.65,(245,245,245),2)
    cv2.rectangle(camera,(0,0),(width,40),(14,22,33),-1)
    cv2.putText(camera,'MuJoCo: rendered robot RGB camera',(10,26),cv2.FONT_HERSHEY_SIMPLEX,.65,(245,245,245),2)
    dashboard=np.full((180,width*2,3),[22,28,38],dtype=np.uint8)
    lines=[
        f't = {log.t:5.1f}s   Camera marker: {"visible" if log.camera_seen else "not detected"}',
        f'EKF estimated pose: x={log.estimate[0]:+.2f}  y={log.estimate[1]:+.2f}  yaw={log.estimate[2]:+.2f} rad',
        f'Simulator truth for evaluation: x={log.truth[0]:+.2f}  y={log.truth[1]:+.2f}  yaw={log.truth[2]:+.2f} rad',
        f'EKF uncertainty (1 sigma): x={log.uncertainty[0]:.3f}m  y={log.uncertainty[1]:.3f}m  yaw={log.uncertainty[2]:.3f}rad',
        f'Estimated-state command: v={log.cmd[0]:.3f} m/s   omega={log.cmd[1]:+.3f} rad/s',
    ]
    for i,line in enumerate(lines):
        cv2.putText(dashboard,line,(18,29+i*31),cv2.FONT_HERSHEY_SIMPLEX,.62,(240,240,240),1,cv2.LINE_AA)
    return np.concatenate([np.concatenate([top,camera],axis=1),dashboard],axis=0)

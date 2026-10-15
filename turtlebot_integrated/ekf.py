"""EKF on SE(2) pose, with noisy wheel odometry prediction and camera range/bearing updates."""
import math
import numpy as np
from .config import Config
from .geometry import wrap, landmark_prediction
from .types import CameraDetection

class PoseEKF:
    def __init__(self, config:Config, initial:np.ndarray):
        self.cfg = config
        self.x = np.array(initial,dtype=float)
        self.P = np.diag([0.09**2,0.09**2,0.12**2])
        self.last_nis: float | None = None

    def predict(self, dl:float, dr:float):
        """Wheel increments (radians), from measured encoders, NOT simulator pose."""
        c=self.cfg
        ds=c.wheel_radius*(dl+dr)*0.5
        dtheta=c.wheel_radius*(dr-dl)/c.wheel_separation
        mid=self.x[2]+0.5*dtheta
        self.x[0] += ds*math.cos(mid)
        self.x[1] += ds*math.sin(mid)
        self.x[2] = wrap(self.x[2]+dtheta)
        F=np.eye(3)
        F[0,2]=-ds*math.sin(mid)
        F[1,2]=ds*math.cos(mid)
        # Encoder uncertainty is mapped into SE(2) process covariance.
        G=np.array([[c.wheel_radius*0.5*math.cos(mid),c.wheel_radius*0.5*math.cos(mid)],
                    [c.wheel_radius*0.5*math.sin(mid),c.wheel_radius*0.5*math.sin(mid)],
                    [-c.wheel_radius/c.wheel_separation,c.wheel_radius/c.wheel_separation]])
        var=(c.encoder_noise_rad**2)+.0001*(abs(dl)+abs(dr))
        Q=G @ (np.eye(2)*var) @ G.T + np.diag([1e-6,1e-6,2e-6])
        self.P=F@self.P@F.T+Q
        self.P=(self.P+self.P.T)*.5

    def update_camera(self, detection:CameraDetection | None, marker_xy:tuple[float,float], noise_scale:float=1):
        self.last_nis=None
        if detection is None:
            return False
        z=np.array([detection.distance,detection.bearing])
        h,H=landmark_prediction(self.x,marker_xy)
        innovation=z-h
        innovation[1]=wrap(innovation[1])
        R=np.diag([(self.cfg.camera_noise_range*noise_scale)**2,
                   (self.cfg.camera_noise_bearing*noise_scale)**2])
        S=H@self.P@H.T+R
        self.last_nis=float(innovation.T@np.linalg.solve(S,innovation))
        # Chi-square 2-D 99.9% threshold; reject implausible visual pose.
        if self.last_nis>13.82:
            return False
        K=np.linalg.solve(S,(self.P@H.T).T).T
        self.x += K@innovation
        self.x[2]=wrap(self.x[2])
        I=np.eye(3)
        self.P=(I-K@H)@self.P@(I-K@H).T+K@R@K.T # Joseph form
        self.P=(self.P+self.P.T)*.5
        return True

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
        self.last_used_gyro = False
        self.last_odom_minus_gyro: float | None = None

    def predict(self, dl:float, dr:float, gyro_delta_yaw: float | None = None):
        """Wheel increments (radians), from measured encoders, NOT simulator pose."""
        c=self.cfg
        # Calibrate encoder-to-body displacement without changing the physical
        # MuJoCo wheel actuators. Truth is NEVER used in this online estimate.
        radius = c.wheel_radius if c.odom_wheel_radius is None else c.odom_wheel_radius
        separation = c.wheel_separation if c.odom_wheel_separation is None else c.odom_wheel_separation
        if radius <= 0 or separation <= 0:
            raise ValueError('Odometry wheel radius and separation must be positive')
        ds=radius*(dl+dr)*0.5
        wheel_dtheta=radius*(dr-dl)/separation
        if not 0 <= c.gyro_yaw_weight <= 1:
            raise ValueError('gyro_yaw_weight must be between 0 and 1')
        self.last_used_gyro=bool(c.use_gyro_yaw and gyro_delta_yaw is not None
                                 and np.isfinite(gyro_delta_yaw))
        blend=c.gyro_yaw_weight if self.last_used_gyro else 0.0
        self.last_odom_minus_gyro=(wrap(wheel_dtheta-gyro_delta_yaw)
                                  if self.last_used_gyro else None)
        # Encoder distance + blended encoder/IMU yaw increments, all measured.
        # Gyro does not provide absolute yaw, so visual updates remain essential.
        valid_gyro_delta=float(gyro_delta_yaw) if self.last_used_gyro else 0.0
        dtheta=(1-blend)*wheel_dtheta+blend*valid_gyro_delta
        mid=self.x[2]+0.5*dtheta
        self.x[0] += ds*math.cos(mid)
        self.x[1] += ds*math.sin(mid)
        self.x[2] = wrap(self.x[2]+dtheta)
        F=np.eye(3)
        F[0,2]=-ds*math.sin(mid)
        F[1,2]=ds*math.cos(mid)
        # Encoder uncertainty is mapped into SE(2) process covariance.
        G=np.array([[radius*0.5*math.cos(mid),radius*0.5*math.cos(mid)],
                    [radius*0.5*math.sin(mid),radius*0.5*math.sin(mid)],
                    [-(1-blend)*radius/separation,(1-blend)*radius/separation]])
        var=(c.encoder_noise_rad**2)+.0001*(abs(dl)+abs(dr))
        Q=G @ (np.eye(2)*var) @ G.T + np.diag([1e-6,1e-6,2e-6])
        if self.last_used_gyro:
            # Angular-rate noise integrated for one sample, with a conservative
            # extra modelling floor. Bias is *not* claimed to be estimated.
            Q[2,2]+=blend**2*((c.gyro_noise_rad_s*c.dt)**2
                                +(c.gyro_bias_rad_s*c.dt)**2+1e-5)
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

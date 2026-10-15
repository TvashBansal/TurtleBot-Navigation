"""ROS-neutral sensor packet adapter and safety limits for hardware. Offline-testable."""
from dataclasses import dataclass
import math
import numpy as np
from .config import Config
from .types import Scan,SensorPacket,CameraDetection

@dataclass
class HardwareStatus:
    timestamp: float
    encoder_timestamp: float | None
    lidar_timestamp: float | None
    camera_timestamp: float | None
    encoder_ready: bool
    scan_ready: bool

class SensorBridge:
    def __init__(self,c:Config):
        self.c=c
        self.encoders=None;self.scan=None;self.camera=None
        self.encoder_at=None;self.scan_at=None;self.camera_at=None
        self.camera_consumed_at=None

    def encoder_update(self,left:float,right:float,stamp:float):
        self.encoders=(float(left),float(right));self.encoder_at=float(stamp)

    def scan_update(self,ranges, angle_min:float,angle_increment:float,range_max:float,stamp:float):
        src=np.asarray(ranges,dtype=float)
        angles=angle_min+np.arange(len(src))*angle_increment
        if len(src)==0:raise ValueError('Empty LaserScan')
        # Remove unknown and out-of-range values. Range scan remains in robot base frame.
        src=np.where(np.isfinite(src),src,self.c.lidar_max_range)
        src=np.clip(src,.03,min(float(range_max),self.c.lidar_max_range))
        self.scan=Scan(src,angles,min(self.c.lidar_max_range,float(range_max)))
        self.scan_at=float(stamp)

    def camera_update(self,detection:CameraDetection | None,stamp:float):
        self.camera=detection;self.camera_at=float(stamp)

    def packet(self):
        if self.encoders is None or self.scan is None: return None
        fresh_cam=self.camera if self.camera_at != self.camera_consumed_at else None
        self.camera_consumed_at=self.camera_at
        return SensorPacket(*self.encoders,self.scan,fresh_cam)

    def status(self,now:float):
        return HardwareStatus(now,self.encoder_at,self.scan_at,self.camera_at,
                              self.encoder_at is not None and now-self.encoder_at<.35,
                              self.scan_at is not None and now-self.scan_at<.35)

class SafetyGate:
    """Stop on missing fresh encoders/scan and unseen marker in last metre of approach."""
    def __init__(self,c:Config):
        self.c=c
        self.last_marker_seen=None

    def apply(self,v:float,w:float,status:HardwareStatus,estimate:np.ndarray | None,
              camera_seen:bool,enable_motion:bool):
        if camera_seen:self.last_marker_seen=status.timestamp
        if not enable_motion:return 0.0,0.0,'dry-run: motion disabled'
        if not status.encoder_ready or not status.scan_ready:
            return 0.0,0.0,'missing/stale encoder or LiDAR message'
        if estimate is None:return 0.0,0.0,'missing estimated pose'
        distance=math.hypot(estimate[0]-self.c.dock_pose[0],estimate[1]-self.c.dock_pose[1])
        if distance<.75 and (self.last_marker_seen is None or status.timestamp-self.last_marker_seen>2.0):
            return 0.0,0.0,'no recent visual marker at docking approach'
        return float(np.clip(v,0,.12)),float(np.clip(w,-.8,.8)),'command limited'

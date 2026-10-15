"""Types; 'truth' is exclusively used by simulators, experiment metrics and rendering."""
from dataclasses import dataclass, field
import numpy as np

@dataclass
class Pose2:
    x: float
    y: float
    theta: float

    def array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)

@dataclass
class Scan:
    ranges: np.ndarray
    angles: np.ndarray
    max_range: float

@dataclass
class CameraDetection:
    distance: float
    bearing: float
    corners: np.ndarray | None = None

@dataclass
class SensorPacket:
    encoder_left: float
    encoder_right: float
    scan: Scan
    camera: CameraDetection | None
    gyro_z_rad_s: float | None = None  # optional IMU yaw angular velocity, base_link frame

@dataclass
class StepLog:
    t: float
    truth: list[float]
    estimate: list[float]
    uncertainty: list[float]
    cmd: list[float]
    target: list[float]
    camera_seen: bool
    distance_to_goal: float
    collision: bool
    innovation_nis: float | None = None
    gyro_z_rad_s: float | None = None
    gyro_used: bool = False
    camera_update_accepted: bool = False
    odom_minus_gyro_delta_yaw_rad: float | None = None
    nav_stop_reason: str | None = None
    front_clearance_m: float | None = None
    waypoint_index: int | None = None
    target_heading_error_rad: float | None = None

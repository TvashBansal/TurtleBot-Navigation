from dataclasses import dataclass, field
from pathlib import Path
import yaml

@dataclass
class Config:
    wheel_radius: float = 0.033
    wheel_separation: float = 0.160
    robot_radius: float = 0.125
    dt: float = 0.08
    max_speed: float = 0.22
    max_omega: float = 1.6
    lidar_beams: int = 72
    lidar_max_range: float = 3.2
    lidar_noise: float = 0.015
    encoder_noise_rad: float = 0.003
    camera_noise_range: float = 0.045
    camera_noise_bearing: float = 0.028
    camera_dropout: float = 0.025
    camera_max_range: float = 2.8
    camera_half_fov_deg: float = 43.0
    camera_width: int = 640
    camera_height: int = 480
    camera_marker_size: float = 0.22
    camera_height_m: float = 0.200
    camera_offset_x_m: float = 0.105
    camera_fovy_deg: float = 55.0
    map_resolution: float = 0.12
    map_inflation: float = 0.16
    max_duration: float = 100.0
    dock_tolerance: float = 0.18
    seed: int = 7
    marker_xy: tuple[float,float] = (1.82, 0.0)
    dock_pose: tuple[float,float,float] = (1.35, 0.0, 0.0)
    start_pose: tuple[float,float,float] = (-1.75, -1.10, 0.05)
    # Ordered goal: approach near marker, then dock.
    waypoints: tuple[tuple[float,float],...] = ((-0.85,-0.75),(0.15,-0.30),(1.05,0.0),(1.35,0.0))

    @classmethod
    def load(cls, path: str | Path | None = None) -> 'Config':
        c = cls()
        if path:
            payload = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
            if not isinstance(payload, dict):
                raise ValueError('Config YAML must be mapping')
            for k,v in payload.items():
                if not hasattr(c,k):
                    raise ValueError(f'Unknown config key: {k}')
                setattr(c,k,tuple(map(tuple,v)) if k=='waypoints' else tuple(v) if k in {'marker_xy','dock_pose','start_pose'} else v)
        return c

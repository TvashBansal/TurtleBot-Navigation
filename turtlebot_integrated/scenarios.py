"""Environment geometry is allowed only inside simulated sensors/physics/metrics."""
from dataclasses import dataclass
import numpy as np
from .types import Pose2
from .geometry import point_rect_distance

@dataclass
class Scenario:
    name: str
    obstacles: list[tuple[float,float,float,float]]
    start: Pose2
    wheel_slip: float = 0.0
    camera_dropout_extra: float = 0.0
    noise_multiplier: float = 1.0
    disturbance_at: float | None = None
    # World-frame impulse: applied as a finite-duration external force/torque
    # in MuJoCo. A controller cannot read this schedule or the true pose.
    disturbance_duration_s: float = 0.40
    disturbance_force_y_n: float = 0.90
    disturbance_yaw_torque_nm: float = 0.025
    # World boundaries 5 x 4 meters. Keep visual marker fixed (1.82, 0).
    bounds: tuple[float,float,float,float] = (-2.4, 2.4, -2.0, 2.0)

    def hit(self, x:float,y:float, radius:float) -> bool:
        xmin,xmax,ymin,ymax=self.bounds
        if x-radius<=xmin or x+radius>=xmax or y-radius<=ymin or y+radius>=ymax:
            return True
        return any(point_rect_distance(x,y,r)<radius for r in self.obstacles)


def get_scenario(name: str, seed: int=7) -> Scenario:
    if name not in {'nominal','noise','blocked','disturbance','camera_loss'}:
        raise ValueError(f'Unknown scenario: {name}')
    # Obstacles deliberately do not obstruct the final docking approach.
    obstacles=[(-0.5,-0.15,-1.55,-0.72), (0.40,0.75,0.42,1.35)]
    if name=='blocked':
        obstacles += [(0.25,0.60,-0.70,-0.34)]
    return Scenario(name=name,obstacles=obstacles,start=Pose2(-1.75,-1.10,0.05),
        wheel_slip=.045 if name in {'disturbance','noise'} else 0,
        camera_dropout_extra=.35 if name=='camera_loss' else 0,
        noise_multiplier=2.5 if name=='noise' else 1.0,
        disturbance_at=9.0 if name=='disturbance' else None)


def disturbance_wrench_world(scenario: Scenario, time_s: float) -> np.ndarray:
    """MuJoCo body wrench [Fx,Fy,Fz,Mx,My,Mz] in the WORLD frame.

    Returns zero for all non-disturbance scenarios and when outside the
    specified finite-duration interval. Never exposed to the controller.
    """
    start = scenario.disturbance_at
    if start is None or not (start <= time_s < start + scenario.disturbance_duration_s):
        return np.zeros(6, dtype=float)
    return np.array([0.0, scenario.disturbance_force_y_n, 0.0,
                     0.0, 0.0, scenario.disturbance_yaw_torque_nm], dtype=float)

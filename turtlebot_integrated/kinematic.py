"""Offline TEST DOUBLE. This is not the compulsory MuJoCo simulation."""
import math
import numpy as np
from .config import Config
from .geometry import wrap,ray_rect
from .scenarios import Scenario
from .types import CameraDetection,Pose2,Scan,SensorPacket

class KinematicTestBackend:
    def __init__(self,cfg:Config,scenario:Scenario,seed:int=7):
        self.cfg=cfg;self.scenario=scenario;self.rng=np.random.default_rng(seed)
        self.truth=scenario.start.array()
        self.left=0.0;self.right=0.0;self.time=0.0
        self.collision=False
        self.last_cmd=(0.0,0.0)

    def get_truth(self):return self.truth.copy()

    def _ray(self,x,y,angle):
        dx,dy=math.cos(angle),math.sin(angle)
        xmin,xmax,ymin,ymax=self.scenario.bounds
        boxes=self.scenario.obstacles+[
            (xmin-.1,xmin,ymin,ymax),(xmax,xmax+.1,ymin,ymax),
            (xmin,xmax,ymin-.1,ymin),(xmin,xmax,ymax,ymax+.1)]
        return min(ray_rect(x,y,dx,dy,box) for box in boxes)

    def observe(self):
        c=self.cfg;s=self.scenario
        angles=np.linspace(-math.pi,math.pi,c.lidar_beams,endpoint=False)
        d=np.array([self._ray(self.truth[0],self.truth[1],self.truth[2]+a) for a in angles])
        d=np.minimum(d,c.lidar_max_range)
        d=np.clip(d+self.rng.normal(0,c.lidar_noise*s.noise_multiplier,len(d)),0.03,c.lidar_max_range)
        scan=Scan(d,angles,c.lidar_max_range)
        dx=c.marker_xy[0]-self.truth[0];dy=c.marker_xy[1]-self.truth[1]
        dist=math.hypot(dx,dy);bearing=wrap(math.atan2(dy,dx)-self.truth[2])
        visible=dist<c.camera_max_range and abs(bearing)<math.radians(c.camera_half_fov_deg) and self.truth[0]<c.marker_xy[0]
        # Occlusion: if ray intersects an obstacle closer than marker, marker cannot be seen.
        if visible and self._ray(self.truth[0],self.truth[1],self.truth[2]+bearing)<dist-.05:
            visible=False
        if self.rng.random()<c.camera_dropout+s.camera_dropout_extra:visible=False
        det=None
        if visible:
            det=CameraDetection(float(dist+self.rng.normal(0,c.camera_noise_range*s.noise_multiplier)),
                wrap(bearing+self.rng.normal(0,c.camera_noise_bearing*s.noise_multiplier)))
        return SensorPacket(self.left+self.rng.normal(0,c.encoder_noise_rad),
            self.right+self.rng.normal(0,c.encoder_noise_rad),scan,det)

    def step(self,v:float,w:float):
        c=self.cfg;s=self.scenario;dt=c.dt
        v=float(np.clip(v,-c.max_speed,c.max_speed));w=float(np.clip(w,-c.max_omega,c.max_omega))
        # Encoders integrate commanded wheel speeds; slip reduces the actual displacement.
        vl=v-.5*c.wheel_separation*w;vr=v+.5*c.wheel_separation*w
        self.left+=vl*dt/c.wheel_radius;self.right+=vr*dt/c.wheel_radius
        slip=s.wheel_slip
        v_actual=v*(1-slip)
        w_actual=w*(1-.3*slip)
        middle=self.truth[2]+w_actual*dt*.5
        proposed=self.truth.copy()
        proposed[0]+=v_actual*dt*math.cos(middle)
        proposed[1]+=v_actual*dt*math.sin(middle)
        proposed[2]=wrap(proposed[2]+w_actual*dt)
        if s.disturbance_at is not None and self.time<s.disturbance_at<=self.time+dt:
            proposed[1]+=.10
            proposed[2]=wrap(proposed[2]+.12)
        if s.hit(float(proposed[0]),float(proposed[1]),c.robot_radius):
            self.collision=True
        else:self.truth=proposed
        self.last_cmd=(v,w);self.time+=dt

    def render_rgb(self):
        return None

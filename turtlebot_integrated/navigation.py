"""Planner/controller consumes only estimated state and sensor packet."""
import math
import numpy as np
from .control import LQRTracker
from .config import Config
from .geometry import wrap
from .mapping import OccupancyMap
from .types import SensorPacket

class Navigator:
    def __init__(self,c:Config):
        self.c=c
        self.controller=LQRTracker(c)
        self.map=OccupancyMap(c)
        self.idx=0
        self.last_plan_t=-1e9
        self.path=[]
        self.done=False

    def tick(self,t:float,pose:np.ndarray,packet:SensorPacket):
        self.map.update(pose,packet.scan)
        waypoints=self.c.waypoints
        while self.idx<len(waypoints)-1 and math.dist(pose[:2],waypoints[self.idx])<.24:
            self.idx+=1
            self.path=[]
        gx,gy=waypoints[self.idx]
        last=self.idx==len(waypoints)-1
        if last and math.dist(pose[:2],(gx,gy))<self.c.dock_tolerance and abs(wrap(pose[2]-self.c.dock_pose[2]))<.18:
            self.done=True
            return (0.0,0.0),(gx,gy)
        if t-self.last_plan_t>1.0 or not self.path:
            self.last_plan_t=t
            planned=self.map.plan((float(pose[0]),float(pose[1])),(gx,gy))
            if planned:self.path=planned
        # Look ahead along estimated path, not simulator ground truth.
        target=(gx,gy)
        if self.path:
            for p in self.path:
                if math.dist(p,pose[:2])>.33:
                    target=p
                    break
        if last and math.dist(pose[:2],(gx,gy))<.35:
            target=(gx,gy)
        heading=self.c.dock_pose[2] if last and math.dist(pose[:2],(gx,gy))<.3 else None
        v,w=self.controller.track(pose,target,heading)
        # Real /scan equivalent: brake for obstacles in the driving corridor.
        scan=packet.scan
        frontal=np.abs(scan.angles)<0.40
        ranges=scan.ranges[frontal]
        nearest=float(np.min(ranges)) if len(ranges) else math.inf
        if nearest < self.c.robot_radius+.13:
            v=0.0
            # turn toward the safer side of the scan and replan next cycle
            left=scan.ranges[(scan.angles>.2)&(scan.angles<1.1)]
            right=scan.ranges[(scan.angles<-.2)&(scan.angles> -1.1)]
            leftmean=float(np.mean(left)) if len(left) else 0
            rightmean=float(np.mean(right)) if len(right) else 0
            w=0.7 if leftmean>=rightmean else -0.7
            self.last_plan_t=-1e9
        elif nearest<self.c.robot_radius+.32:
            v*=max(0.1,(nearest-self.c.robot_radius-.13)/.19)
        return (v,w),target

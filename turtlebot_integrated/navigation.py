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
        self.last_stop_reason = 'not_started'
        self.last_front_clearance_m = float('inf')
        self.last_target_bearing_rad = 0.0
        self.last_planner_status = 'not_started'
        self.last_path_nodes = 0
        self.last_visible_nodes = 0
        self.last_target_distance_m = 0.0
        self.last_goal_region_fallback = False

    def tick(self,t:float,pose:np.ndarray,packet:SensorPacket):
        self.map.update(pose,packet.scan)
        waypoints=self.c.waypoints
        while self.idx<len(waypoints)-1 and math.dist(pose[:2],waypoints[self.idx])<self.c.nav_intermediate_goal_radius_m:
            self.idx+=1
            self.path=[]
        gx,gy=waypoints[self.idx]
        last=self.idx==len(waypoints)-1
        if last and math.dist(pose[:2],(gx,gy))<self.c.dock_tolerance and abs(wrap(pose[2]-self.c.dock_pose[2]))<.18:
            self.done=True
            self.last_stop_reason='goal_reached'
            self.last_planner_status='goal_reached'
            return (0.0,0.0),(gx,gy)
        if t-self.last_plan_t>1.0 or not self.path:
            self.last_plan_t=t
            start_xy = (float(pose[0]), float(pose[1]))
            planned=self.map.plan(start_xy,(gx,gy))
            self.last_goal_region_fallback = False
            if not planned and not last and self.c.nav_intermediate_goal_region_fallback:
                # A* can fail even when the nominal waypoint cell is free:
                # the cell may be surrounded by inflated obstacles. Try a
                # bounded, FREE and REACHABLE cell within the intermediate
                # waypoint's configured capture radius. The region planner
                # applies the same obstacle mask and no-corner-cutting rule.
                # If the region is disconnected/blocked, it returns [] and
                # navigation safely holds; no direct-goal shortcut is used.
                planned=self.map.plan_to_goal_region(
                    start_xy,(gx,gy),self.c.nav_intermediate_goal_radius_m)
                self.last_goal_region_fallback = bool(planned)
            # Discard stale routes whenever replanning fails; don't continue
            # traversing a path invalidated by newly observed obstacles.
            self.path=planned
        # Select a visible lookahead ON the A* path. A far waypoint chosen
        # merely by distance can cut diagonally across inflated obstacles.
        # Visibility is checked against the LiDAR-built map (NOT truth).
        # With no A* route, do not drive directly at the goal through an
        # unverified corridor; wait for the next replan.
        target=(float(pose[0]),float(pose[1]))
        self.last_planner_status='no_astar_route' if not self.path else 'path_found'
        self.last_path_nodes=len(self.path)
        self.last_visible_nodes=0
        if self.path:
            mask=self.map.obstacles()
            # The first 3x3 cells were deliberately cleared by A* when the
            # estimated robot begins within its own inflated obstacle halo.
            # Apply the same exception for visibility testing near that start.
            start_cell=self.map.xy_to_cell(float(pose[0]),float(pose[1]))
            for yy in range(max(0,start_cell[1]-1),min(mask.shape[0],start_cell[1]+2)):
                for xx in range(max(0,start_cell[0]-1),min(mask.shape[1],start_cell[0]+2)):
                    mask[yy,xx]=False
            visible=[]
            for p in self.path:
                if math.dist(p,pose[:2]) < self.map.res*.35:
                    continue
                if not self.map.segment_clear(tuple(pose[:2]),p,mask):
                    # A* path is ordered; never skip around a blocked segment
                    # to reach a farther target.
                    break
                visible.append(p)
            self.last_visible_nodes=len(visible)
            if visible:
                # Prefer an onward grid node outside the controller's capture
                # radius, but only along a *directly clear* segment. This avoids
                # needlessly selecting a node that the controller already regards
                # as reached; never jump through a mapped obstacle.
                within=[p for p in visible if math.dist(p,pose[:2])<=self.c.nav_lookahead_m]
                candidates=within if within else [visible[0]]
                beyond=[p for p in candidates if math.dist(p,pose[:2])>=self.c.nav_local_capture_radius_m]
                target=beyond[-1] if beyond else candidates[-1]
                self.last_planner_status=(('goal_region_path' if self.last_goal_region_fallback else 'path_following')
                                          if beyond else 'near_path_node')
            else:
                # Fail safely if the map has no visible onward path.
                # Force replanning on the next tick; do not drive through it.
                self.path=[]
                self.last_plan_t=-1e9
                target=(float(pose[0]),float(pose[1]))
                self.last_planner_status='no_visible_onward_path'
        if last and math.dist(pose[:2],(gx,gy))<.35:
            # Last-stage docking uses goal position/heading, not a map
            # shortcut far across obstacles.
            target=(gx,gy)
            self.last_planner_status='final_docking'
        heading=self.c.dock_pose[2] if last and math.dist(pose[:2],(gx,gy))<.3 else None
        self.last_target_distance_m=math.dist(target,pose[:2])
        self.last_target_bearing_rad=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2])
        v,w=self.controller.track(pose,target,heading)
        self.last_stop_reason='controller_turn' if v<1e-6 and abs(w)>1e-6 else (
            'controller_hold' if v<1e-6 else 'moving')
        # Real /scan equivalent: brake for obstacles in the driving corridor.
        scan=packet.scan
        frontal=np.abs(scan.angles)<0.40
        ranges=scan.ranges[frontal]
        nearest=float(np.min(ranges)) if len(ranges) else math.inf
        self.last_front_clearance_m=nearest
        if nearest < self.c.robot_radius+.13:
            v=0.0
            # turn toward the safer side of the scan and replan next cycle
            left=scan.ranges[(scan.angles>.2)&(scan.angles<1.1)]
            right=scan.ranges[(scan.angles<-.2)&(scan.angles> -1.1)]
            leftmean=float(np.mean(left)) if len(left) else 0
            rightmean=float(np.mean(right)) if len(right) else 0
            w=0.7 if leftmean>=rightmean else -0.7
            self.last_plan_t=-1e9
            self.last_stop_reason='safety_brake'
        elif nearest<self.c.robot_radius+.32:
            v*=max(0.1,(nearest-self.c.robot_radius-.13)/.19)
            if self.last_stop_reason=='moving':
                self.last_stop_reason='safety_slowed'
        return (v,w),target

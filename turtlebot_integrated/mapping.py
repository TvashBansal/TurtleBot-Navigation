"""LiDAR-only occupancy map (no simulator obstacle/ground-truth reads by the planner)."""
import heapq
import math
import numpy as np
from scipy.ndimage import binary_dilation
from .config import Config
from .types import Scan

class OccupancyMap:
    def __init__(self,cfg:Config,bounds=(-2.4,2.4,-2.0,2.0)):
        self.c=cfg
        self.bounds=bounds
        self.res=cfg.map_resolution
        self.width=int(math.ceil((bounds[1]-bounds[0])/self.res))
        self.height=int(math.ceil((bounds[3]-bounds[2])/self.res))
        self.log_odds=np.zeros((self.height,self.width), dtype=float)

    def xy_to_cell(self,x,y):
        return (int((x-self.bounds[0])/self.res), int((y-self.bounds[2])/self.res))

    def cell_to_xy(self,c):
        return (self.bounds[0]+(c[0]+.5)*self.res,self.bounds[2]+(c[1]+.5)*self.res)

    def inbounds(self,c):
        return 0<=c[0]<self.width and 0<=c[1]<self.height

    def update(self,pose:np.ndarray,scan:Scan):
        x,y,theta=pose
        for dist,ang in zip(scan.ranges,scan.angles):
            if not np.isfinite(dist):continue
            hit=dist < scan.max_range-.07
            length=min(float(dist),scan.max_range)
            a=theta+ang
            steps=max(1,int(length/(self.res*.6)))
            for j in range(steps):
                d=length*j/steps
                cell=self.xy_to_cell(x+d*math.cos(a),y+d*math.sin(a))
                if self.inbounds(cell):
                    self.log_odds[cell[1],cell[0]]-=0.13
            if hit:
                cell=self.xy_to_cell(x+length*math.cos(a),y+length*math.sin(a))
                if self.inbounds(cell):self.log_odds[cell[1],cell[0]]+=0.8
        np.clip(self.log_odds,-3.5,5,out=self.log_odds)

    def obstacles(self):
        r=max(1,int(math.ceil(self.c.map_inflation/self.res)))
        cells=binary_dilation(self.log_odds>1.6,iterations=r)
        cells[0,:]=True;cells[-1,:]=True;cells[:,0]=True;cells[:,-1]=True
        return cells

    def plan(self,start:tuple[float,float],goal:tuple[float,float]):
        s=self.xy_to_cell(*start);g=self.xy_to_cell(*goal)
        if not (self.inbounds(s) and self.inbounds(g)):return []
        mask=self.obstacles()
        # Ignore local occupied estimate at start; safety is handled by reactive LiDAR braking.
        for yy in range(max(0,s[1]-1),min(self.height,s[1]+2)):
            for xx in range(max(0,s[0]-1),min(self.width,s[0]+2)):
                mask[yy,xx]=False
        if mask[g[1],g[0]]:
            return []
        def heuristic(c):return math.hypot(c[0]-g[0],c[1]-g[1])
        q=[(heuristic(s),0,s)]
        cost={s:0};parent={}
        neighbors=[(dx,dy,math.hypot(dx,dy)) for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
        while q:
            _, current_cost,c=heapq.heappop(q)
            if current_cost>cost.get(c,math.inf)+1e-9:continue
            if c==g:break
            for dx,dy,step_cost in neighbors:
                n=(c[0]+dx,c[1]+dy)
                if not self.inbounds(n) or mask[n[1],n[0]]:continue
                if dx and dy and (mask[c[1],n[0]] or mask[n[1],c[0]]):continue
                val=cost[c]+step_cost+max(self.log_odds[n[1],n[0]],0)*.2
                if val<cost.get(n,math.inf):
                    cost[n]=val;parent[n]=c;heapq.heappush(q,(val+heuristic(n),val,n))
        if g not in cost:return []
        path=[g]
        while path[-1]!=s:path.append(parent[path[-1]])
        path.reverse()
        # Pick every other cell; waypoint follower can smooth between these.
        return [self.cell_to_xy(c) for c in path]

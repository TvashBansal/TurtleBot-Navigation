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

    def segment_clear(self, start: tuple[float, float], end: tuple[float, float],
                      occupied: np.ndarray | None = None) -> bool:
        """Check direct line of sight against the *estimated*, inflated occupancy grid.

        Used only to select a local target on an already computed A* path.
        The geometry originates from LiDAR, never from simulator ground truth.
        """
        mask = self.obstacles() if occupied is None else occupied
        dx, dy = end[0] - start[0], end[1] - start[1]
        count = max(1, int(math.ceil(math.hypot(dx, dy) / (self.res * 0.25))))
        for i in range(count + 1):
            t = i / count
            cell = self.xy_to_cell(start[0] + t*dx, start[1] + t*dy)
            if not self.inbounds(cell) or mask[cell[1], cell[0]]:
                return False
        return True

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


    def plan_to_goal_region(self, start: tuple[float, float],
                            goal: tuple[float, float],
                            radius_m: float) -> list[tuple[float, float]]:
        """A* to a *free, reachable cell within* a waypoint capture region.

        Only for intermediate waypoints, not the precise charging-station goal.
        A cell is accepted only if its centre falls inside the region and is
        outside the same inflated LiDAR occupancy mask used by normal A*.
        In particular, a blocked nominal goal is never cleared or traversed.
        """
        if not math.isfinite(radius_m) or radius_m <= 0:
            return []
        s = self.xy_to_cell(*start)
        if not self.inbounds(s) or not self.inbounds(self.xy_to_cell(*goal)):
            return []
        mask = self.obstacles()
        # Preserve the same local start exception as plan(). We must not
        # discard the robot's position solely because an obstacle halo grew
        # around it; direct LiDAR braking still prevents unsafe motion.
        for yy in range(max(0, s[1]-1), min(self.height, s[1]+2)):
            for xx in range(max(0, s[0]-1), min(self.width, s[0]+2)):
                mask[yy, xx] = False

        # Require a cell-centre to be *strictly* within the configured
        # capture distance, leaving a small margin for discretisation.
        capture_radius = max(0., radius_m - 0.015)
        acceptable = set()
        g = self.xy_to_cell(*goal)
        extent = math.ceil(radius_m / self.res) + 2
        for yy in range(max(0, g[1]-extent), min(self.height, g[1]+extent+1)):
            for xx in range(max(0, g[0]-extent), min(self.width, g[0]+extent+1)):
                if not mask[yy, xx] and math.dist(self.cell_to_xy((xx,yy)), goal) <= capture_radius:
                    acceptable.add((xx, yy))
        if not acceptable:
            return []

        # A* with an exact Euclidean-distance lower bound to the set of
        # accepted cells. No corner cutting or skipping across obstacles.
        # Candidate sets are tiny for intermediate capture radii (~0.24m).
        def heuristic(cell):
            return min(math.hypot(cell[0]-item[0], cell[1]-item[1])
                       for item in acceptable)
        q = [(heuristic(s), 0., s)]
        cost = {s: 0.}
        parent = {}
        endpoint = None
        while q:
            _, current_cost, cell = heapq.heappop(q)
            if current_cost > cost.get(cell, math.inf) + 1e-9:
                continue
            if cell in acceptable:
                endpoint = cell
                break
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    if not (dx or dy):
                        continue
                    nxt = (cell[0]+dx, cell[1]+dy)
                    if not self.inbounds(nxt) or mask[nxt[1], nxt[0]]:
                        continue
                    if dx and dy and (mask[cell[1], nxt[0]] or mask[nxt[1], cell[0]]):
                        continue
                    new_cost = cost[cell] + math.hypot(dx,dy) + max(self.log_odds[nxt[1],nxt[0]],0)*.2
                    if new_cost < cost.get(nxt, math.inf):
                        cost[nxt] = new_cost
                        parent[nxt] = cell
                        heapq.heappush(q, (new_cost + heuristic(nxt),new_cost,nxt))
        if endpoint is None:
            return []
        path = [endpoint]
        while path[-1] != s:
            path.append(parent[path[-1]])
        path.reverse()
        return [self.cell_to_xy(cell) for cell in path]

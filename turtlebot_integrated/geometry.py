import math
import numpy as np

def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def ray_rect(x: float, y: float, dx: float, dy: float,
             rect: tuple[float,float,float,float]) -> float:
    """Distance to first ray/AABB intersection; inf if none. Rectangle xmin,xmax,ymin,ymax."""
    xmin, xmax, ymin, ymax = rect
    lo, hi = -math.inf, math.inf
    for p, direction, a, b in ((x, dx, xmin, xmax), (y, dy, ymin, ymax)):
        if abs(direction) < 1e-12:
            if not a <= p <= b:
                return math.inf
        else:
            t1, t2 = (a-p)/direction, (b-p)/direction
            lo = max(lo, min(t1,t2))
            hi = min(hi, max(t1,t2))
    if hi < max(lo, 0.0):
        return math.inf
    return max(lo, 0.0) if lo >= 0 else hi

def point_rect_distance(x: float, y: float, r: tuple[float,float,float,float]) -> float:
    xmin,xmax,ymin,ymax = r
    return math.hypot(max(xmin-x,0,x-xmax), max(ymin-y,0,y-ymax))

def landmark_prediction(x: np.ndarray, landmark_xy: tuple[float,float]):
    dx, dy = landmark_xy[0]-x[0], landmark_xy[1]-x[1]
    q = max(dx*dx+dy*dy, 1e-12)
    d = math.sqrt(q)
    return np.array([d, wrap(math.atan2(dy,dx)-x[2])]), np.array([
        [-dx/d, -dy/d, 0],
        [dy/q, -dx/q, -1],
    ])

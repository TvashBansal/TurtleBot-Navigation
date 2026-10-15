"""Time-varying discrete LQR for differential drive local tracking errors."""
import math
import numpy as np
from scipy.linalg import solve_discrete_are
from .config import Config
from .geometry import wrap

class LQRTracker:
    def __init__(self, cfg:Config):
        self.c=cfg
        self.Q=np.diag([4.0,18.0,10.0])
        self.R=np.diag([1.5,1.1])

    def track(self, estimate:np.ndarray, target:tuple[float,float],
              final_heading:float | None=None) -> tuple[float,float]:
        dx,dy=target[0]-estimate[0],target[1]-estimate[1]
        distance=math.hypot(dx,dy)
        if distance < 0.13 and final_heading is not None:
            return 0.0,float(np.clip(2.0*wrap(final_heading-estimate[2]),-self.c.max_omega,self.c.max_omega))
        if distance < 0.08:
            return 0.0,0.0
        theta_target=math.atan2(dy,dx)
        angle=wrap(theta_target-estimate[2])
        # Turn-in-place if target outside heading cone, rather than driving into obstacles.
        if abs(angle)>1.05:
            return 0.0,float(np.clip(2.0*angle,-self.c.max_omega,self.c.max_omega))
        vref=min(self.c.max_speed*.78,max(.07,distance*.55))
        wref=0.0
        # Error in body reference frame.
        ex=math.cos(estimate[2])*dx+math.sin(estimate[2])*dy
        ey=-math.sin(estimate[2])*dx+math.cos(estimate[2])*dy
        e=np.array([ex,ey,angle])
        A=np.array([[0,wref,0],[-wref,0,vref],[0,0,0]])
        B=np.array([[-1.0,0],[0,0],[0,-1]])
        Ad=np.eye(3)+self.c.dt*A
        Bd=self.c.dt*B
        try:
            P=solve_discrete_are(Ad,Bd,self.Q,self.R)
            K=np.linalg.solve(self.R+Bd.T@P@Bd,Bd.T@P@Ad)
            delta=-K@e
            v,vomega=vref+delta[0], wref+delta[1]
        except np.linalg.LinAlgError:
            v,vomega=vref+0.7*ex,1.5*angle
        # Reverse is disabled for safety; rotate instead.
        v=float(np.clip(v,0,self.c.max_speed))
        vomega=float(np.clip(vomega,-self.c.max_omega,self.c.max_omega))
        return v,vomega

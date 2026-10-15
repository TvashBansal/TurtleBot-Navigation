"""Reproduce control/observation ranks for the report without a MuJoCo runtime."""
import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.geometry import landmark_prediction

def ranks(vref=.16,marker=(1.82,0.0),x=np.array([0.,0.,0.])):
    A=np.array([[0,0,0],[0,0,vref],[0,0,0]])
    B=np.array([[-1,0],[0,0],[0,-1]],float)
    _,H=landmark_prediction(x,marker)
    controllability=np.concatenate([B,A@B,np.linalg.matrix_power(A,2)@B],axis=1)
    observability=np.concatenate([H,H@A,H@np.linalg.matrix_power(A,2)],axis=0)
    return np.linalg.matrix_rank(controllability),np.linalg.matrix_rank(observability),A,B,H

if __name__=='__main__':
    for v in [0,.16]:
        c,o,*_=ranks(vref=v)
        print(f'vref = {v:.2f} m/s: controllability rank={c}/3, camera local observability rank={o}/3')

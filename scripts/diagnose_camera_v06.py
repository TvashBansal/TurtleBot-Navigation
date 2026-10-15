"""A/B: standard vs multiscale ArUco on identical actual MuJoCo frames.

Offline diagnostic only: manually chosen physical poses are used to obtain
repeatable images. Simulator true pose is NOT supplied to the vision estimator.
"""
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')
os.environ.setdefault('MPLBACKEND', 'Agg')

import cv2
import numpy as np
from turtlebot_integrated.camera import marker_from_rgb
from turtlebot_integrated.config import Config
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario


def sample_pose(sim, x, y, yaw):
    sim.data.qpos[:7] = [x,y,0.088,math.cos(yaw/2),0,0,math.sin(yaw/2)]
    sim.data.qvel[:] = 0
    sim.mj.mj_forward(sim.model,sim.data)
    return sim.render_rgb('front_camera')


def main():
    c=Config.load('configs/mujoco_v06_camera_only.yaml')
    sim=MuJoCoBackend(c,get_scenario('nominal',7),seed=7)
    poses=[('center',.5,0,0),('left_unrotated',.5,-.3,0),
           ('right_unrotated',.5,.3,0),('near_aimed',1.0,0,0),
           ('far_aimed',-.4,0,0)]
    result=[]
    out=Path('outputs/diagnostics/camera_v06')
    out.mkdir(parents=True,exist_ok=True)
    try:
        for name,x,y,yaw in poses:
            rgb=sample_pose(sim,x,y,yaw)
            cv2.imwrite(str(out/f'{name}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            entries={'pose':[x,y,yaw]}
            for mode in ('standard','multiscale'):
                c.camera_detection_mode=mode
                d=marker_from_rgb(rgb,c)
                entries[mode]={'detected':d is not None}
                if d is not None:
                    entries[mode].update(range_m=round(d.distance,4),bearing_rad=round(d.bearing,4))
            entries['case']=name
            result.append(entries)
            print(name,entries['standard'],entries['multiscale'])
    finally:
        sim.close()
    payload={'backend':'mujoco', 'same_rgb_frames_for_both_detectors':True,
             'truth_for_offline_camera_positioning_only':True,
             'no_docking_success_claimed':True,'cases':result}
    (out/'comparison.json').write_text(json.dumps(payload,indent=2)+'\n',encoding='utf-8')
    print('Saved:',out)


if __name__=='__main__':
    main()

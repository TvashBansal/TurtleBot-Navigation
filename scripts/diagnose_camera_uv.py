"""Verify MuJoCo visual marker detection and EKF landmark-update geometry.

Run from repo root: MUJOCO_GL=osmesa python scripts/diagnose_camera_uv.py
Writes output-only debug images/report; does not change the saved original marker.
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
from turtlebot_integrated.ekf import PoseEKF
from turtlebot_integrated.geometry import wrap
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario


def main():
    cfg=Config()
    out=Path('outputs/diagnostics/camera_uv')
    out.mkdir(parents=True, exist_ok=True)
    sim=MuJoCoBackend(cfg,get_scenario('nominal',7),seed=7)
    results=[]
    # All three poses face the marker; left/right offsets verify bearing signs.
    probes=[('center',0.50,0.0,0.0),('marker_left',0.50,-0.30,0.0),
            ('marker_right',0.50,0.30,0.0)]
    try:
        for name,x,y,theta in probes:
            sim.data.qpos[:7]=[x,y,0.088,math.cos(theta/2),0,0,math.sin(theta/2)]
            sim.mj.mj_forward(sim.model,sim.data)
            frame=sim.render_rgb('front_camera')
            cv2.imwrite(str(out/f'{name}.png'),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
            detected=marker_from_rgb(frame,cfg)
            expected_distance=math.hypot(cfg.marker_xy[0]-x,cfg.marker_xy[1]-y)
            expected_bearing=wrap(math.atan2(cfg.marker_xy[1]-y,cfg.marker_xy[0]-x)-theta)
            result={'pose':[x,y,theta],'detected':detected is not None,
                    'expected_range_m':round(expected_distance,4),
                    'expected_bearing_rad':round(expected_bearing,4)}
            if detected is not None:
                result.update(measured_range_m=round(detected.distance,4),
                              measured_bearing_rad=round(detected.bearing,4))
                ekf=PoseEKF(cfg,np.array([x,y,theta]))
                accepted=ekf.update_camera(detected,cfg.marker_xy)
                result['ekf_update_accepted']=bool(accepted)
                result['innovation_nis']=None if ekf.last_nis is None else round(ekf.last_nis,4)
            print(f'{name}: {result}',flush=True)
            results.append(result)
    finally:
        sim.close()
    (out/'camera_uv_report.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    print('Report and images:',out)
    if not results[0]['detected']:
        raise SystemExit('ERROR: center marker still not visible in the unflipped MuJoCo frame.')
    print('Center marker detected in ORIGINAL frame; verify range/bearing and EKF flags above.')


if __name__=='__main__':
    main()

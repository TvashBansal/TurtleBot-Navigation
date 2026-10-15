"""Compare encoder yaw versus IMU-aided yaw in a controlled MuJoCo physics test.

Simulator ground truth is used EXCLUSIVELY for offline scoring, never filtering.
This trial is a pre-navigation sensor validation, NOT completed docking evidence.

Run: MUJOCO_GL=osmesa python scripts/diagnose_gyro_motion.py
"""
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MUJOCO_GL','osmesa')
os.environ.setdefault('MPLBACKEND','Agg')

import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.ekf import PoseEKF
from turtlebot_integrated.geometry import wrap
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario


def main():
    cfg=Config.load('configs/mujoco_gyro_candidate.yaml')
    baseline_cfg=Config.load('configs/mujoco_odom_candidate.yaml')
    sim=MuJoCoBackend(cfg,get_scenario('nominal',7),seed=7,record_camera=False)
    wheel_ekf=PoseEKF(baseline_cfg,np.array(cfg.start_pose))
    aided_ekf=PoseEKF(cfg,np.array(cfg.start_pose))
    records=[]
    previous=None
    # Physically executed: stationary spin both ways and curved drive.
    segments=[('spin_ccw',0.,.7,6.),('spin_cw',0.,-.7,6.),
              ('curved_drive',.07,.35,5.)]
    try:
        for name,v,w,duration in segments:
            for _ in range(round(duration/cfg.dt)):
                packet=sim.observe()
                if previous is not None:
                    dl=packet.encoder_left-previous[0]
                    dr=packet.encoder_right-previous[1]
                    wheel_ekf.predict(dl,dr)
                    aided_ekf.predict(dl,dr,packet.gyro_z_rad_s*cfg.dt)
                previous=(packet.encoder_left,packet.encoder_right)
                sim.step(v,w)
                truth=sim.get_truth()
                records.append(dict(t=round(sim.time,3),segment=name,
                    truth=truth.tolist(),wheel=wheel_ekf.x.tolist(),gyro_aided=aided_ekf.x.tolist(),
                    gyro_rad_s=packet.gyro_z_rad_s,
                    encoder_vs_gyro_delta_yaw=aided_ekf.last_odom_minus_gyro))
    finally:
        sim.close()
    # Compare to truth only after sensor filtering is finished.
    # One final unprocessed step creates <= 0.08-second lag in both filters.
    heading=lambda key: np.array([wrap(r[key][2]-r['truth'][2]) for r in records])
    metrics={}
    for label,key in [('encoder_only','wheel'),('gyro_aided','gyro_aided')]:
        e=heading(key)
        ep=np.array([math.dist(r[key][:2],r['truth'][:2]) for r in records])
        metrics[label]=dict(heading_rmse_rad=round(float(np.sqrt(np.mean(e**2))),4),
                            final_abs_heading_error_rad=round(float(abs(e[-1])),4),
                            position_rmse_m=round(float(np.sqrt(np.mean(ep**2))),4))
    folder=Path('outputs/diagnostics/gyro_v05')
    folder.mkdir(parents=True,exist_ok=True)
    report={'backend':'mujoco','type':'open_loop_physics_probe',
            'uses_truth_only_for_offline_evaluation':True,
            'segments':[{'label':n,'v':v,'w':w,'duration_s':d} for n,v,w,d in segments],
            'metrics':metrics,'success_not_claimed':True}
    (folder/'gyro_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    (folder/'gyro_trajectory.json').write_text(json.dumps(records)+'\n')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,4))
    ax.plot([r['t'] for r in records],heading('wheel'),label='encoder-only yaw error')
    ax.plot([r['t'] for r in records],heading('gyro_aided'),label='gyro-aided yaw error')
    ax.set(xlabel='Simulation time (s)',ylabel='Wrapped yaw estimation error (rad)',title='Open-loop yaw-rate sensor diagnostic')
    ax.legend();ax.grid(alpha=.2);fig.tight_layout()
    fig.savefig(folder/'heading_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps(report,indent=2))
    print('Saved:',folder)

if __name__=='__main__':main()

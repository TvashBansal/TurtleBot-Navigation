"""Compare unchanged v0.11 MuJoCo body physics with an OPTIONAL front support.

Open-loop physics test only, NOT a navigation success claim. The true body
rotation, LiDAR ray world directions, and contacts are examined for diagnosis;
they are never supplied to the EKF, planner, controller or sensor readings.

Run: MUJOCO_GL=osmesa MPLBACKEND=Agg python scripts/diagnose_v12_chassis_support.py
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario

OUT = Path('outputs/diagnostics/v12_chassis_support')
OUT.mkdir(parents=True, exist_ok=True)


def attitude(sim):
    r = np.asarray(sim.data.xmat[sim.body_id], dtype=float).reshape(3, 3)
    roll = math.degrees(math.atan2(r[2, 1], r[2, 2]))
    pitch = math.degrees(math.atan2(-r[2, 0], math.hypot(r[2, 1], r[2, 2])))
    return roll, pitch


def inspect(sim, pkt):
    front = np.flatnonzero(np.abs(pkt.scan.angles) < .40)
    floor_matches = 0
    lowest_ray = None
    close = 0
    for beam in front:
        sensor = sim.ray_sids[int(beam)]
        site_id = int(sim.model.sensor_objid[sensor])
        xyz = np.asarray(sim.data.site_xpos[site_id])
        d = np.asarray(sim.data.site_xmat[site_id]).reshape(3,3)[:, 2]
        measured = float(pkt.scan.ranges[beam])
        if measured < .445:
            close += 1
        predicted_floor = float(xyz[2] / -d[2]) if d[2] < -1e-5 and xyz[2] > 0 else None
        if predicted_floor is not None and abs(predicted_floor - measured) <= .05:
            floor_matches += 1
        if lowest_ray is None or measured < lowest_ray:
            lowest_ray = measured
    roll, pitch = attitude(sim)
    support_contacts = 0
    for i in range(sim.data.ncon):
        contact = sim.data.contact[i]
        for gid in (contact.geom1, contact.geom2):
            name = sim.mj.mj_id2name(sim.model,sim.mj.mjtObj.mjOBJ_GEOM,int(gid))
            if name == 'front_support':
                support_contacts += 1
                break
    return {'t_s': round(float(sim.time),3), 'roll_deg': round(roll,3),
            'pitch_deg': round(pitch,3),
            'min_front_range_m': round(float(lowest_ray),4),
            'front_ray_close_count': close,
            'front_ray_floor_matches':floor_matches,
            'front_support_contacts':support_contacts}


def execute(flag):
    cfg = Config.load('configs/mujoco_v12_front_support_probe.yaml')
    cfg.mujoco_front_support_enabled = flag
    cfg.lidar_noise = 0.0  # keep physical rays comparable; no sensor suppression
    sim = MuJoCoBackend(cfg,get_scenario('nominal',7),seed=7,record_camera=False)
    records = []
    # Same commanded physical inputs in both models.
    schedule = ((0.0,0.0,1.0),(.10,0.0,2.0),(0.0,.6,2.0),(.07,.30,3.0))
    try:
        for v,w,duration in schedule:
            for _ in range(round(duration/cfg.dt)):
                records.append(inspect(sim,sim.observe()))
                sim.step(v,w)
        records.append(inspect(sim,sim.observe()))
    finally:
        sim.close()
    arr = np.array([abs(r['pitch_deg']) for r in records])
    return {'front_support_enabled':flag,
            'duration_s':records[-1]['t_s'],
            'max_abs_pitch_deg':round(float(arr.max()),3),
            'pitch_abs_p50_deg':round(float(np.median(arr)),3),
            'pitch_abs_p95_deg':round(float(np.percentile(arr,95)),3),
            'time_with_front_range_under_0_445_fraction':round(float(np.mean([r['min_front_range_m']<.445 for r in records])),3),
            'scans_with_any_floor_match':sum(r['front_ray_floor_matches']>0 for r in records),
            'support_contact_samples':sum(r['front_support_contacts']>0 for r in records),
            'samples':len(records), 'records':records}


def main():
    baseline=execute(False)
    candidate=execute(True)
    report={'purpose':'Physics-only comparison of chassis support; ground truth used for OFFLINE evaluation',
            'baseline':baseline,'candidate':candidate,
            'important':'Additional caster changes the physical model; it must be documented and tested before use in assessment.'}
    path=OUT/'comparison.json'
    path.write_text(json.dumps(report,indent=2)+'\n')
    fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for label,d in [('original',baseline),('with_support',candidate)]:
        ts=[r['t_s'] for r in d['records']]
        axs[0].plot(ts,[r['pitch_deg'] for r in d['records']],label=label)
        axs[1].plot(ts,[r['min_front_range_m'] for r in d['records']],label=label)
    axs[0].set_ylabel('True body pitch (degrees)')
    axs[1].set_ylabel('Minimum front LiDAR range (m)')
    axs[1].set_xlabel('MuJoCo time (s)')
    axs[1].axhline(.445,color='grey',linestyle=':',label='safety slowdown threshold')
    for ax in axs:
        ax.grid(alpha=.2)
        ax.legend()
    fig.tight_layout()
    fig.savefig(OUT/'pitch_and_lidar.png',dpi=160)
    plt.close(fig)
    print('=== v0.12 MUJOCO CHASSIS ATTITUDE / FLOOR-RAY COMPARISON ===')
    for label,d in [('original',baseline),('front_support',candidate)]:
        print(f'{label}: max_pitch={d["max_abs_pitch_deg"]:.2f} deg, '
              f'p95_pitch={d["pitch_abs_p95_deg"]:.2f} deg, '
              f'near_front_scans={d["time_with_front_range_under_0_445_fraction"]:.1%}, '
              f'floor_matches={d["scans_with_any_floor_match"]}/{d["samples"]}, '
              f'front_support_contact_samples={d["support_contact_samples"]}')
    print('Full report:',path)
    print('Plot:',OUT/'pitch_and_lidar.png')
    print('Physical model comparison ONLY. No docking success claimed.')

if __name__=='__main__':
    main()

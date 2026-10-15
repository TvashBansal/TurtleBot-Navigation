import argparse
import json
from pathlib import Path
from .config import Config
from .experiment import run_trial

SCENARIOS=['nominal','noise','blocked','disturbance','camera_loss']

def main():
    parser=argparse.ArgumentParser(description='Track A: estimated-state TurtleBot3 navigation')
    sub=parser.add_subparsers(dest='command',required=True)
    run=sub.add_parser('run',help='Run one MuJoCo experiment (or explicit offline test double)')
    run.add_argument('--backend',choices=['mujoco','kinematic'],default='mujoco')
    run.add_argument('--scenario',choices=SCENARIOS,default='nominal')
    run.add_argument('--seed',type=int,default=7)
    run.add_argument('--config',type=str)
    run.add_argument('--out',type=Path,default=Path('outputs/nominal'))
    run.add_argument('--video',action='store_true',help='Record MuJoCo rendered camera MP4')
    suite=sub.add_parser('suite',help='Run repeated trials across scenarios')
    suite.add_argument('--backend',choices=['mujoco','kinematic'],default='mujoco')
    suite.add_argument('--config',type=str)
    suite.add_argument('--out',type=Path,default=Path('outputs/suite'))
    suite.add_argument('--seeds',type=int,default=3)
    args=parser.parse_args()
    cfg=Config.load(args.config)
    if args.command=='run':
        result=run_trial(cfg,args.backend,args.scenario,args.seed,args.out,args.video)
        print(json.dumps(result,indent=2))
    elif args.command=='suite':
        results=[]
        for scenario in SCENARIOS:
            for seed in range(args.seeds):
                out=args.out/scenario/f'seed_{seed}'
                results.append(run_trial(cfg,args.backend,scenario,seed,out))
                print(f'{scenario} / seed {seed}: {results[-1]["success"]} ({results[-1]["duration_s"]:.1f} s)')
        import numpy as np
        agg={'backend':args.backend,'count':len(results),
             'success_rate':float(np.mean([r['success'] for r in results])),
             'collision_rate':float(np.mean([r['collision'] for r in results])),
             'mean_position_error_m':float(np.mean([r['position_error_m'] for r in results])),
             'mean_rmse_xy_m':float(np.mean([np.hypot(r['rmse_x_m'],r['rmse_y_m']) for r in results])),
             'scenario_results':results}
        args.out.mkdir(parents=True,exist_ok=True)
        (args.out/'aggregate.json').write_text(json.dumps(agg,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in agg.items() if k!='scenario_results'},indent=2))

if __name__=='__main__':main()

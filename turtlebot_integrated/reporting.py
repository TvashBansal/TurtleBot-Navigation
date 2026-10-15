"""Figure generation uses simulator truth only as ground truth for assessment plots."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def plot_trial(logs,summary:dict,outdir:Path):
    t=np.array([x.t for x in logs])
    truth=np.array([x.truth for x in logs]);est=np.array([x.estimate for x in logs]);sig=np.array([x.uncertainty for x in logs])
    if len(t)==0:return
    fig,ax=plt.subplots(figsize=(8,6))
    ax.plot(truth[:,0],truth[:,1],label='Simulator truth (evaluation only)')
    ax.plot(est[:,0],est[:,1],label='EKF estimate',linestyle='--')
    ax.scatter([1.35],[0],label='Dock target',marker='x')
    ax.set(xlabel='World x [m]',ylabel='World y [m]',title='Closed-loop navigation')
    ax.grid(alpha=.3);ax.legend();ax.set_aspect('equal',adjustable='box');fig.tight_layout()
    fig.savefig(outdir/'trajectory.png',dpi=150);plt.close(fig)
    fig,axs=plt.subplots(3,1,figsize=(9,8),sharex=True)
    names=['x [m]','y [m]','heading [rad]']
    errs=truth-est;errs[:,2]=(errs[:,2]+np.pi)%(2*np.pi)-np.pi
    for j,a in enumerate(axs):
        a.plot(t,errs[:,j],label='True - estimated error')
        a.plot(t,2*sig[:,j],linestyle='--',label='+2σ')
        a.plot(t,-2*sig[:,j],linestyle='--',label='-2σ')
        a.set_ylabel(names[j]);a.grid(alpha=.3);a.legend(loc='upper right',fontsize=7)
    axs[-1].set_xlabel('Time [s]')
    fig.suptitle('EKF performance and estimated uncertainty')
    fig.tight_layout();fig.savefig(outdir/'estimation.png',dpi=150);plt.close(fig)
    fig,axs=plt.subplots(2,1,figsize=(9,5),sharex=True)
    cmds=np.array([x.cmd for x in logs])
    axs[0].plot(t,cmds[:,0]);axs[0].set_ylabel('v [m/s]')
    axs[1].plot(t,cmds[:,1]);axs[1].set_ylabel('omega [rad/s]')
    axs[1].set_xlabel('Time [s]')
    for a in axs:a.grid(alpha=.3)
    fig.tight_layout();fig.savefig(outdir/'controls.png',dpi=150);plt.close(fig)

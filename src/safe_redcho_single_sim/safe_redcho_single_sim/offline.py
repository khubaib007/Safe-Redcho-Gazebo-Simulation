"""Numerical SE(3) screening of the single-bank mission; no ROS/Gazebo claim."""
import argparse
import json
from pathlib import Path
import numpy as np
from safe_redcho_swarm.core.config import Params
from safe_redcho_swarm.core.quadrotor import integrate_crazyflie_se3
from .local_controller import LocalController, initial_plant_state
from .history import allocate_history,store_agent,record_global,metadata,performance_summary,export_csv
from . import scenario


def run(duration, output, *, plots=False, csv=True, disturbance=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    h=allocate_history(duration);agents={i:LocalController(i) for i in range(1,7)}
    state=initial_plant_state();state[0][2]-=9.81e-6;state[1][2]-=.00981
    previous=np.zeros((3,6));params=Params();completed=0
    meta=metadata(duration,'Numerical SE(3) midpoint plant plus simple floor proxy; no ROS/Gazebo execution')
    meta['bounded_disturbance_test']=disturbance
    error=None;summary=None
    try:
        for k,t in enumerate(h['time']):
            prepared={i:a.prepare(k+1,float(t),state,previous) for i,a in agents.items()}
            bounds=np.array([prepared[i][1] for i in range(1,7)])
            for i in range(6,0,-1):
                a=agents[i];d=a.finish({j:prepared[j][0] for j in a.neighbors},bounds)
                store_agent(h,k,i,d)
            record_global(h,k,state);completed=k+1
            if k%500==0:print(f't={t:.2f}s | {scenario.phase(t)} | min pair={h["min_dist_hist"][k]:.4f}m',flush=True)
            p,v,R,w=state
            for i in range(6):
                p[:,i],v[:,i],R[:,:,i],w[:,i]=integrate_crazyflie_se3(p[:,i],v[:,i],R[:,:,i],w[:,i],h['u_motors_hist'][:,i,k],params,scenario.DT)
            if disturbance and 28. <= t < 30.:
                v[:,2]+=.01*np.array([.12,-.08,.08]);v[:,4]+=.01*np.array([-.10,.06,-.06])
            contact=p[2]<.017425;p[2,contact]=.017425;v[2,contact]=np.maximum(v[2,contact],0.)
            previous=h['u_safe_hist'][...,k].copy()
        summary=performance_summary(h,meta)
        if not all(summary['checks'].values()):raise RuntimeError('Single-bank numerical performance checks failed')
    except Exception as exc:error=type(exc).__name__+': '+str(exc)
    kept={key:a[...,:completed] for key,a in h.items()}
    success=error is None
    np.savez_compressed(output/('simulation_data.npz' if success else 'partial_simulation_data.npz'),**kept)
    (output/'metadata.json').write_text(json.dumps(meta,indent=2))
    if summary:(output/'performance_summary.json').write_text(json.dumps(summary,indent=2))
    if csv:export_csv(kept,output)
    if plots and success:
        from .figures import make_plots
        make_plots(kept,meta,output)
    report=dict(status='PASS' if success else 'FAIL',completed_cycles=completed,
        expected_cycles=len(h['time']),error=error,checks=summary['checks'] if summary else {},
        scope=meta['plant_backend'],architecture=scenario.ARCHITECTURE,banks_per_agent=1)
    (output/'mission_status.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    if not success:raise RuntimeError(error)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=scenario.MISSION_DURATION)
    parser.add_argument('--output',type=Path,default=Path('results/single_position_offline'))
    parser.add_argument('--no-plots',action='store_true');parser.add_argument('--no-csv',action='store_true')
    parser.add_argument('--disturbance',action='store_true')
    args=parser.parse_args();run(args.duration,args.output,plots=not args.no_plots,csv=not args.no_csv,disturbance=args.disturbance)


if __name__=='__main__':main()

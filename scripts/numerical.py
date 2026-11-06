"""Run six local controllers in numerical SE(3) feedback, without ROS."""
import os,sys
import argparse,json,time
from pathlib import Path
import numpy as np
from safe_redcho_distributed.local_controller import LocalController,initial_plant_state
from safe_redcho_distributed import scenario
from safe_redcho_distributed.mission_history import allocate_history,record_global_diagnostics,mission_metadata,scenario_performance_summary
from safe_redcho_swarm.core.config import Params
from safe_redcho_swarm.core.quadrotor import integrate_crazyflie_se3


if __name__=='__main__' and json.loads((Path(__file__).resolve().parents[1]/'release.json').read_text())['variant']=='single-position':
    os.execv(sys.executable,[sys.executable,'-s','-m','safe_redcho_single_sim.offline',*[x for x in sys.argv[1:] if x!='--plots']])

def run(duration,output,plots=False):
    if not np.isfinite(duration) or duration<0 or abs(duration/.01-round(duration/.01))>1e-8:
        raise ValueError('duration must be a nonnegative multiple of 0.01 seconds')
    output=Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('Output directory must be new or empty: '+str(output))
    output.mkdir(parents=True,exist_ok=True)
    h=allocate_history(duration);agents={i:LocalController(i) for i in range(1,7)}
    state=initial_plant_state();previous=np.zeros((3,6));params=Params();completed=0;began=time.monotonic()
    state[0][2]-=9.81e-6;state[1][2]-=.00981
    formation=hasattr(agents[1],'formation_packet')
    try:
        for k,t in enumerate(h['time']):
            prepared={i:a.prepare(k+1,float(t),state,previous) for i,a in agents.items()}
            packets={i:a.formation_packet() for i,a in agents.items()} if formation else None
            bounds=np.array([prepared[i][1] for i in range(1,7)])
            for i in range(6,0,-1):
                a=agents[i];args=[{j:prepared[j][0] for j in a.neighbors},bounds]
                if formation:args.append({j:packets[j] for j in a.neighbors})
                for key,value in a.finish(*args).items():h[key][...,i-1,k]=value
            record_global_diagnostics(h,k,state);completed=k+1
            if k%500==0:print(f't={t:.2f}s, minimum pair spacing={h["min_dist_hist"][k]:.4f} m',flush=True)
            p,v,R,w=state
            for i in range(6):p[:,i],v[:,i],R[:,:,i],w[:,i]=integrate_crazyflie_se3(p[:,i],v[:,i],R[:,:,i],w[:,i],h['u_motors_hist'][:,i,k],params,.01)
            contact=p[2]<.017425;p[2,contact]=.017425;v[2,contact]=np.maximum(v[2,contact],0.)
            previous=h['u_safe_hist'][...,k].copy()
        if not all(np.isfinite(a).all() for a in h.values()):raise RuntimeError('Nonfinite history')
        meta=mission_metadata(duration)
        meta['plant_backend']='Numerical SE(3) midpoint integrator with a unilateral floor proxy'
        meta['validation_backend']='Numerical feedback only; no ROS/Gazebo execution'
        summary=scenario_performance_summary(h,meta)
        np.savez_compressed(output/'simulation_data.npz',**h)
        (output/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
        (output/'performance_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        if not all(summary['checks'].values()):raise RuntimeError('Performance checks failed: '+str(summary['checks']))
        if plots:
            from safe_redcho_distributed.scenario_figures import make_plots
            make_plots(h,meta,output)
        report=dict(status='PASS',variant='formation' if formation else 'velocity',cycles=completed,duration_s=duration,wall_s=time.monotonic()-began,checks=summary['checks'],scope=meta['validation_backend'])
    except BaseException as exc:
        np.savez_compressed(output/'partial_simulation_data.npz',**{key:a[...,:completed] for key,a in h.items()})
        report=dict(status='FAIL',completed_cycles=completed,error=type(exc).__name__+': '+str(exc),scope='Numerical feedback only')
        (output/'mission_status.json').write_text(json.dumps(report,indent=2)+'\n')
        raise
    (output/'mission_status.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--duration',type=float,default=45.);p.add_argument('--output',type=Path,default=Path('results')/('numerical_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())));p.add_argument('--plots',action='store_true');a=p.parse_args();run(a.duration,a.output,a.plots)

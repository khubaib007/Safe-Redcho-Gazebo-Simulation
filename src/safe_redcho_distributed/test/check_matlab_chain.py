"""Check the six-process computations against the original vectorized equations.

Recorded physical states are verification inputs only. This test does not run
as part of the mission and does not supply commands to agents.
"""
import sys
from pathlib import Path
import json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed import scenario
from safe_redcho_distributed.local_controller import INITIAL_SOC
from safe_redcho_swarm.core.config import Params,Battery,CBF
from safe_redcho_swarm.core.battery import compute_vocv,speed_governor,voltage_aware_accel_bound
from safe_redcho_swarm.core.reference_filters import svf_step,levant_step
from safe_redcho_swarm.core.redcho import build_redcho_matrices,redcho_state_derivative
from safe_redcho_swarm.core.cbf_qp import solve_decentralized_hocbf_qp
from safe_redcho_swarm.core.quadrotor import crazyflie_geometric_controller,integrate_crazyflie_se3

def run(file):
    with np.load(file,allow_pickle=False) as f:h={k:f[k] for k in f.files}
    params,batt,cbf=Params(),Battery(),CBF(sample_time=.01)
    Z=np.zeros((3,3,6));Zl=Z.copy();X=Z.copy();Xl=Z.copy()
    soc=INITIAL_SOC.copy();Vp=np.zeros(6);voltage=compute_vocv(soc);resistance=np.full(6,batt.R_int_nom)
    _,G=build_redcho_matrices(np.full(3,.15));adj=np.zeros((6,6),dtype=int)
    for i in range(6):adj[i,(i-1)%6]=adj[i,(i+1)%6]=1
    E=[np.eye(3)/(scenario.OBSTACLE_RADIUS+scenario.OBSTACLE_DESIGN_CLEARANCE)**2 for _ in range(2)]
    R=np.repeat(np.eye(3)[:,:,None],6,axis=2);w=np.zeros((3,6));maxima={};fault=False
    def compare(key,value,k):
        delta=float(np.max(np.abs(value-h[key][...,k])))
        maxima[key]=max(maxima.get(key,0.),delta)
        assert delta<1e-7,(key,k,delta)
    for k,t in enumerate(h['time']):
        p,v=h['p_hist'][...,k],h['v_hist'][...,k]
        if t>=scenario.FORWARD_START+12 and not fault:soc[1]=.32;resistance[1]*=1.5;fault=True
        speed=speed_governor(soc,batt)
        raw=np.zeros((3,6))
        if t>=scenario.FORWARD_START:
            tau=t-scenario.FORWARD_START
            heading=np.array([np.cos(.04*tau),np.sin(.04*tau),.05*np.sin(.2*tau)])
            raw=heading[:,None]/np.linalg.norm(heading)*speed
        Z,snap=svf_step(Z,raw,2.,.01);Zl=levant_step(Zl,raw,27.43,.01)
        Y=Z-np.einsum('ab,dbi->dai',G,X);Yl=Zl-np.einsum('ab,dbi->dai',G,Xl)
        X+=.01*redcho_state_derivative(X,Y,adj,np.full(3,.15),1.2,np.array([2.,1.4,.8]),27.43,.04)
        Xl+=.01*redcho_state_derivative(Xl,Yl,adj,np.full(3,.15),1.2,np.array([2.,1.4,.8]),27.43,.04)
        err=v-Y[:,0,:]
        nominal=Y[:,1,:]-4.5*err-.45*np.tanh(err)
        if t<scenario.FORWARD_START:
            target,vd,ad=scenario.startup_reference(t)
            nominal=ad[:,None]+6*(target-p)+4*(vd[:,None]-v)
        limits=np.array([voltage_aware_accel_bound(voltage[i],params,batt,cbf.max_design_accel) for i in range(6)])
        previous=np.zeros((3,6)) if k==0 else h['u_safe_hist'][...,k-1]
        safe=np.zeros((3,6));motors=np.zeros((4,6));attitude=np.zeros(6)
        infos=[]
        for i in range(6):
            safe[:,i],info=solve_decentralized_hocbf_qp(i,nominal[:,i],previous,p,v,limits[:,1],np.full(6,.4),scenario.INTERAGENT_DESIGN_DISTANCE,scenario.OBSTACLE_CENTERS,E,2.8,3.2,.8,1.2,cbf)
            infos.append(info)
            motors[:,i],attitude[i]=crazyflie_geometric_controller(R[:,:,i],w[:,i],v[:,i],safe[:,i],params,limits[i,0])
        power=params.c_prop*motors.sum(axis=0)**1.5+params.P_avionics
        current=power/np.maximum(voltage,2.5)
        values=dict(SoC_hist=soc,V_batt_hist=voltage,v_safe_hist=speed,raw_ref_hist=raw,F_max_hist=limits[:,0],u_max_hist=limits[:,1],
            U_hist=Z,Y_hist=Y,U_levant_hist=Zl,Y_levant_hist=Yl,svf_snap_hist=snap,u_safe_hist=safe,u_motors_hist=motors,
            P_elec_hist=power,I_batt_hist=current,attitude_error_hist=attitude,
            motor_utilization_hist=motors.max(axis=0)/np.maximum(limits[:,0],1e-9),
            accel_utilization_hist=np.max(np.abs(safe),axis=0)/np.maximum(limits[:,1],1e-9),
            slew_utilization_hist=np.max(np.abs(safe-previous),axis=0)/.08,
            qp_correction_hist=np.linalg.norm(safe-nominal,axis=0),plant_agent_err_hist=np.linalg.norm(err,axis=0))
        for field,key in [('phase1_slack','qp_phase1_slack_hist'),('max_violation','qp_violation_hist'),('mode','qp_mode_hist'),('row_margin','row_actuator_margin_hist'),('recursive_margin','recursive_feas_margin_hist')]:
            values[key]=np.array([info[field] for info in infos])
        for key,value in values.items():compare(key,value,k)
        soc=np.clip(soc-.01*current/(3600*batt.Q_nom),.01,1.)
        Vp+=.01*(-Vp/(batt.R_p*batt.C_p)+current/batt.C_p)
        voltage=np.maximum(2.5,compute_vocv(soc)-Vp-current*resistance)
        for i in range(6):
            _,_,R[:,:,i],w[:,i]=integrate_crazyflie_se3(p[:,i],v[:,i],R[:,:,i],w[:,i],h['u_motors_hist'][:,i,k],params,.01)
        if k%1000==0:print('verified',k,'cycles; max',max(maxima.values()),flush=True)
    report={'cycles':len(h['time']),'quantities':len(maxima),'tolerance':1e-7,'max_absolute_difference':maxima}
    Path(file).with_name('matlab_chain_equivalence.json').write_text(json.dumps(report,indent=2))
    print('PASS: vectorized source computation matches distributed computation',max(maxima.values()))

if __name__=='__main__':run(sys.argv[1])

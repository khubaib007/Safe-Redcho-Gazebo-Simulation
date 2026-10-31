"""Baseline-compatible histories and diagnostics; no control outputs are computed here."""
from dataclasses import asdict
import numpy as np
from scipy.integrate import quad
from safe_redcho_swarm.core.config import Params, Battery, CBF
from safe_redcho_swarm.core.battery import speed_governor
from safe_redcho_swarm.core.reference_filters import compute_svf_iss_certificate
from safe_redcho_swarm.core.redcho import redcho_strict_kernel
from safe_redcho_swarm.core.quadrotor import ellipsoid_radial_clearance
from .local_controller import INITIAL_SOC
from . import scenario


def allocate_history(duration):
    N=6;steps=int(round(duration/.01))+1;h={}
    shapes = {
        'p_hist':(3,N), 'v_hist':(3,N), 'v_est_hist':(3,N), 'u_motors_hist':(4,N),
        'raw_ref_hist':(3,N), 'svf_snap_hist':(3,N), 'u_safe_hist':(3,N),
        'U_hist':(3,3,N), 'Y_hist':(3,3,N), 'U_avg_hist':(3,3),
        'U_levant_hist':(3,3,N), 'Y_levant_hist':(3,3,N), 'U_levant_avg_hist':(3,3),
        'min_phys_obs_dist':(2,), 'redcho_order_err_svf':(3,), 'redcho_order_err_levant':(3,)}
    agent_keys = ['SoC_hist','V_batt_hist','v_safe_hist','v_mag_hist','u_max_hist','F_max_hist','P_elec_hist',
                  'I_batt_hist','qp_correction_hist','disturbance_bound_hist','model_mismatch_hist',
                  'qp_phase1_slack_hist','qp_violation_hist','qp_mode_hist','row_actuator_margin_hist',
                  'recursive_feas_margin_hist','accel_utilization_hist','slew_utilization_hist',
                  'motor_utilization_hist','attitude_error_hist','redcho_agent_err_hist',
                  'plant_agent_err_hist','cohesion_agent_err_hist']
    shapes.update({key:(N,) for key in agent_keys})
    shapes.update({key:() for key in ['min_dist_hist','svf_regularizer_hist','redcho_observer_err_hist',
                                     'plant_tracking_err_hist','vel_consensus_err_hist']})
    for key, shape in shapes.items(): h[key] = np.zeros((*shape, steps))
    h['time']=np.arange(steps)*.01
    return h


def mission_metadata(duration):
    params,batt,cbf=Params(),Battery(),CBF(sample_time=.01)
    omega=2.;gamma=np.full(3,.15)
    c_w=np.poly(-gamma)[1:][::-1]-np.array([omega**3,3*omega**2,3*omega])
    K_w=quad(lambda tau:abs(redcho_strict_kernel(tau,omega,c_w)),0,np.inf,epsabs=1e-12,epsrel=1e-10)[0]
    s0=speed_governor(INITIAL_SOC,batt)
    L_ic=0.0  # Initial observer reference is zero for all six agents
    L_R_star=L_ic+2*.65*(omega**3+K_w)
    return dict(params=asdict(params),battery=asdict(batt),cbf=asdict(cbf),
        duration=duration,dt=.01,fault_time=scenario.FORWARD_START+12.,fault_agent_id=2,
        L_svf=float(compute_svf_iss_certificate(omega,.65,.65)['L_svf_vector']),
        K_w=float(K_w),L_ic=float(L_ic),L_R_star=float(L_R_star),L=27.43,
        initial_soc=INITIAL_SOC.tolist(),obstacle_centers=scenario.OBSTACLE_CENTERS.tolist(),
        obstacle_radii=np.full((3,2),scenario.OBSTACLE_RADIUS).tolist(),
        interagent_safety_distance=scenario.INTERAGENT_DISTANCE,obstacle_surface_clearance=scenario.OBSTACLE_CLEARANCE,
        interagent_design_distance=scenario.INTERAGENT_DESIGN_DISTANCE,obstacle_design_clearance=scenario.OBSTACLE_DESIGN_CLEARANCE,
        interagent_buffer=scenario.INTERAGENT_BUFFER,obstacle_buffer=scenario.OBSTACLE_BUFFER,
        initial_positions=scenario.INITIAL_POSITIONS.tolist(),initial_spacing=scenario.INITIAL_SPACING,
        hover_height=scenario.HOVER_HEIGHT,reference="Source normalized heading times SoC-governed speed; no prescribed bypass or flight position feedback",
        forward_start=scenario.FORWARD_START,flight_clock_offset=scenario.FORWARD_START,
        fault_flight_time=12.,startup_controller="Position/velocity tracking only before flight; no flight position feedback",
        solver='SciPy SLSQP; baseline QP formulation with scenario safety thresholds',
        plant_backend='Gazebo Harmonic, native ten-step motor-force backend',
        hardware_target='Crazyflie 2.1+; MATLAB-derived surrogate parameters',
        architecture='Six independent offboard local controllers; ring REDCHO and shared safety context')


def record_global_diagnostics(h,k,state):
    p,v,_,_=state
    h['p_hist'][...,k]=p;h['v_hist'][...,k]=v
    h['v_mag_hist'][...,k]=np.linalg.norm(v,axis=0)
    distances=np.linalg.norm(p[:,:,None]-p[:,None,:],axis=0)
    np.fill_diagonal(distances,np.inf);h['min_dist_hist'][k]=distances.min()
    centers=scenario.OBSTACLE_CENTERS
    for o in range(2):
        h['min_phys_obs_dist'][o,k]=min(ellipsoid_radial_clearance(p[:,i],centers[:,o],np.full(3,scenario.OBSTACLE_RADIUS)) for i in range(6))
    Z,Y=h['U_hist'][...,k],h['Y_hist'][...,k]
    Zl,Yl=h['U_levant_hist'][...,k],h['Y_levant_hist'][...,k]
    h['U_avg_hist'][...,k]=Z.mean(axis=2);h['U_levant_avg_hist'][...,k]=Zl.mean(axis=2)
    h['v_est_hist'][...,k]=Y[:,0,:]
    obs=np.linalg.norm(Y[:,0,:]-Z[:,0,:].mean(axis=1)[:,None],axis=0)
    plant=np.linalg.norm(v-Y[:,0,:],axis=0)
    cohesion=np.linalg.norm(v-v.mean(axis=1)[:,None],axis=0)
    for key,value in [('redcho_agent_err_hist',obs),('plant_agent_err_hist',plant),('cohesion_agent_err_hist',cohesion),
                      ('redcho_observer_err_hist',obs.mean()),('plant_tracking_err_hist',plant.mean()),('vel_consensus_err_hist',cohesion.mean())]:h[key][...,k]=value
    h['redcho_order_err_svf'][:,k]=np.linalg.norm(Y-Z.mean(axis=2)[:,:,None],axis=0).mean(axis=1)
    h['redcho_order_err_levant'][:,k]=np.linalg.norm(Yl-Zl.mean(axis=2)[:,:,None],axis=0).mean(axis=1)
    h['svf_regularizer_hist'][k]=np.max(np.linalg.norm(h['svf_snap_hist'][...,k],axis=0))


import json
def scenario_performance_summary(h, meta):
    fault_mask = h['time'] >= meta['fault_time']
    # Keep the source's full-run TV calculation (the paper describes post-fault TV).
    tv_svf = np.linalg.norm(np.diff(h['U_hist'][:,2,1,:],axis=1),axis=0).sum()
    tv_lev = np.linalg.norm(np.diff(h['U_levant_hist'][:,2,1,:],axis=1),axis=0).sum()
    rms = lambda key: np.sqrt(np.mean(h[key][:,fault_mask]**2,axis=1)).tolist() if fault_mask.any() else [None]*3
    summary = {key:meta[key] for key in ['duration','fault_time','fault_agent_id','L_svf','K_w','L_ic','L_R_star','L']}
    summary.update(max_svf_derivative=float(h['svf_regularizer_hist'].max()),
                   levant_svf_jerk_tv_ratio=float(tv_lev/max(tv_svf,np.finfo(float).eps)),
                   svf_postfault_rms=rms('redcho_order_err_svf'),levant_postfault_rms=rms('redcho_order_err_levant'),
                   min_obstacle_clearance=h['min_phys_obs_dist'].min(axis=1).tolist(),
                   min_interagent_distance=float(h['min_dist_hist'].min()),
                   min_row_margin=float(h['row_actuator_margin_hist'].min()),
                   max_phase1_slack=float(h['qp_phase1_slack_hist'].max()),max_qp_violation=float(h['qp_violation_hist'].max()),
                   phase1_invocations=int(np.count_nonzero(h['qp_mode_hist'])),agent_steps=int(h['qp_mode_hist'].size),
                   max_se3_residual=float(h['model_mismatch_hist'].max()),
                   peak_accel_slew_motor_utilization=[float(h[key].max()) for key in ['accel_utilization_hist','slew_utilization_hist','motor_utilization_hist']],
                   max_mean_attitude_error=float(h['attitude_error_hist'].mean(axis=0).max()),
                   final_observer_error=float(h['redcho_observer_err_hist'][-1]),final_plant_tracking_error=float(h['plant_tracking_err_hist'][-1]),
                   final_cohesion_error=float(h['vel_consensus_err_hist'][-1]))
    airborne=h['p_hist'][2]>scenario.GROUND_COM_HEIGHT+.01
    consecutive=airborne.copy();consecutive[:,0]=False
    consecutive[:,1:]=airborne[:,1:] & airborne[:,:-1]
    residuals=h['model_mismatch_hist']
    summary['max_all_phase_residual']=float(residuals.max())
    summary['max_se3_residual']=float(residuals[consecutive].max()) if consecutive.any() else 0.
    summary['plant_residual_scope']='Consecutive free-flight samples; ground contact introduces support forces'
    summary['checks'] = dict(svf_derivative=summary['max_svf_derivative']<=meta['L_svf']*(1+1e-6),
                            hard_qp_feasibility=summary['max_phase1_slack']<=1e-6 and summary['max_qp_violation']<=1e-6,
                            plant_residual=summary['max_se3_residual']<=.40+1e-6,
                            interagent_safety=summary['min_interagent_distance']>=meta["interagent_safety_distance"]-1e-3,
                            obstacle_clearance=min(summary['min_obstacle_clearance'])>=meta["obstacle_surface_clearance"]-1e-3)
    gate=int(round(scenario.FORWARD_START/.01))
    if len(h['time'])>gate:
        summary['hover_gate_height_error']=float(np.max(np.abs(h['p_hist'][2,:,gate]-scenario.HOVER_HEIGHT)))
        summary['hover_gate_max_speed']=float(np.max(np.linalg.norm(h['v_hist'][...,gate],axis=0)))
        summary['checks']['hover_before_forward']=summary['hover_gate_height_error']<=scenario.HOVER_HEIGHT_TOLERANCE and summary['hover_gate_max_speed']<=scenario.HOVER_SPEED_TOLERANCE
    print('\nCONTRIBUTION-ALIGNED CHARGE-AWARE SWARM PERFORMANCE REPORT')
    print(json.dumps(summary,indent=2))
    return summary

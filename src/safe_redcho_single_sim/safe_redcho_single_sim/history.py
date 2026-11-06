"""Truthful single-bank telemetry, diagnostics and pass criteria, never control."""
import csv
import json
from dataclasses import asdict
from pathlib import Path
import numpy as np
from safe_redcho_swarm.core.config import Params, Battery, CBF
from safe_redcho_swarm.core.reference_filters import compute_svf_iss_certificate
from safe_redcho_swarm.core.quadrotor import ellipsoid_radial_clearance
from . import scenario
from .local_controller import INITIAL_SOC
from .pose_redcho import regularity_certificate
from .protocol import validate_duration

AGENT_SHAPES = {key: () for key in (
    'SoC_hist','V_batt_hist','v_safe_hist','F_max_hist','u_max_hist','model_mismatch_hist',
    'disturbance_bound_hist','P_elec_hist','I_batt_hist','attitude_error_hist',
    'motor_utilization_hist','accel_utilization_hist','slew_utilization_hist','qp_correction_hist',
    'plant_agent_err_hist','qp_phase1_slack_hist','qp_violation_hist','qp_mode_hist',
    'row_actuator_margin_hist','recursive_feas_margin_hist','formation_activation_hist')}
AGENT_SHAPES.update({key: (3,) for key in (
    'raw_ref_hist','svf_snap_hist','u_safe_hist','formation_correction_hist',
    'formation_position_error_hist','formation_velocity_error_hist','nominal_acceleration_hist',
    'flight_velocity_target_hist','formation_target_hist','pose_regularity_hist')})
AGENT_SHAPES.update({key: (3,3) for key in ('U_hist','pose_local_reference_hist','pose_redcho_estimate_hist')})
AGENT_SHAPES['u_motors_hist'] = (4,)
GLOBAL_SHAPES = {key: () for key in (
    'min_dist_hist','svf_regularizer_hist','redcho_observer_err_hist','plant_tracking_err_hist',
    'vel_consensus_err_hist','formation_shape_error_hist','height_spread_hist','hover_height_error_hist',
    'x_alignment_error_hist','lateral_spacing_error_hist','velocity_spread_hist','pose_estimate_spread_hist',
    'height_estimation_error_hist')}
GLOBAL_SHAPES.update(p_hist=(3,6), v_hist=(3,6), v_est_hist=(3,6), v_mag_hist=(6,),
    min_phys_obs_dist=(2,), reference_obstacle_clearance_hist=(2,), obstacle_qp_margin_hist=(2,6),
    obstacle_free_margin_hist=(2,6), pose_redcho_agent_error_hist=(3,6),
    position_redcho_order_error_hist=(3,), position_reference_average_hist=(3,3),
    U_avg_hist=(3,3), redcho_agent_err_hist=(6,), cohesion_agent_err_hist=(6,))


def allocate_history(duration):
    steps = validate_duration(duration)
    h = {k: np.zeros((*shape,6,steps)) for k,shape in AGENT_SHAPES.items()}
    h.update({k: np.zeros((*shape,steps)) for k,shape in GLOBAL_SHAPES.items()})
    h['time'] = np.arange(steps)*scenario.DT
    return h


def store_agent(h, k, agent_id, values):
    if set(values) != set(AGENT_SHAPES):
        raise ValueError('Single-bank diagnostic field mismatch')
    for key,shape in AGENT_SHAPES.items():
        a = np.asarray(values[key],float)
        if a.shape != shape or not np.isfinite(a).all(): raise ValueError('Invalid diagnostic '+key)
        h[key][...,agent_id-1,k] = a


def metadata(duration, backend):
    return dict(duration=duration, dt=scenario.DT, architecture=scenario.ARCHITECTURE,
        implementation_version=scenario.IMPLEMENTATION_VERSION, banks_per_agent=1,
        redcho_state_scalars_per_agent=9, agents=6, directed_ring_links=12,
        params=asdict(Params()), battery=asdict(Battery()), cbf=asdict(CBF()),
        initial_soc=INITIAL_SOC.tolist(), initial_positions=scenario.INITIAL_POSITIONS.tolist(),
        height_proposals=scenario.HEIGHT_PROPOSALS.tolist(), hover_height=scenario.HOVER_HEIGHT,
        fault_time=scenario.FORWARD_START+12.,fault_agent_id=2,fault_flight_time=12.,
        forward_start=scenario.FORWARD_START, formation_position_gain=scenario.FORMATION_POSITION_GAIN.tolist(),
        formation_position_accel_cap=scenario.FORMATION_POSITION_ACCEL_CAP.tolist(),
        formation_ramp_time=scenario.FORMATION_RAMP_TIME,
        obstacle_centers=scenario.OBSTACLE_CENTERS.tolist(), obstacle_radius=scenario.OBSTACLE_RADIUS,
        obstacle_surface_clearance=scenario.OBSTACLE_CLEARANCE, obstacle_design_clearance=scenario.OBSTACLE_DESIGN_CLEARANCE,
        interagent_safety_distance=scenario.INTERAGENT_DISTANCE, interagent_design_distance=scenario.INTERAGENT_DESIGN_DISTANCE,
        position_redcho_L=scenario.POSE_REDCHO_L, position_redcho_theta=scenario.POSE_REDCHO_THETA,
        position_redcho_eps=scenario.POSE_REDCHO_EPS, position_redcho_regularity_certificate=regularity_certificate(),
        L_svf=compute_svf_iss_certificate(2.,.65,.65)['L_svf_vector'],
        reference='Battery velocity -> local third-order SVF -> integrated XY displacement/analytic height -> ONE position REDCHO -> p/v/a control targets',
        redcho_output_orders=['position_m','velocity_mps','acceleration_mps2'],
        removed_ablation='No velocity REDCHO or Levant REDCHO bank; no fabricated velocity-bank/Levant arrays',
        physical_state_scope='All-six safety context; only two ring peers supply REDCHO estimates',
        plant_backend=backend, time_convention='Log pre-step state at k*dt, then apply one dt motor batch; inclusive duration endpoint')


def record_global(h,k,state):
    p,v,_,_ = state; h['p_hist'][...,k]=p; h['v_hist'][...,k]=v
    h['v_mag_hist'][...,k]=np.linalg.norm(v,axis=0)
    q=p-scenario.INITIAL_POSITIONS
    h['formation_shape_error_hist'][k]=np.linalg.norm(q[:,:,None]-q[:,None,:],axis=0).max()
    for key,value in dict(height_spread_hist=np.ptp(p[2]),hover_height_error_hist=np.abs(p[2]-scenario.HOVER_HEIGHT).max(),
        x_alignment_error_hist=np.ptp(p[0]),lateral_spacing_error_hist=np.abs(np.diff(p[1])-np.diff(scenario.INITIAL_POSITIONS[1])).max(),
        velocity_spread_hist=np.linalg.norm(v[:,:,None]-v[:,None,:],axis=0).max()).items():h[key][k]=value
    distances=np.linalg.norm(p[:,:,None]-p[:,None,:],axis=0);np.fill_diagonal(distances,np.inf)
    h['min_dist_hist'][k]=distances.min()
    local, estimate = h['pose_local_reference_hist'][...,k], h['pose_redcho_estimate_hist'][...,k]
    truth=local.mean(axis=2)  # Logging oracle only, never transmitted to agents.
    errors=np.linalg.norm(estimate-truth[:,:,None],axis=0)
    h['position_reference_average_hist'][...,k]=truth
    h['pose_redcho_agent_error_hist'][...,k]=errors
    h['position_redcho_order_error_hist'][...,k]=errors.mean(axis=1)
    h['pose_estimate_spread_hist'][k]=np.linalg.norm(estimate[:,0,:,None]-estimate[:,0,None,:],axis=0).max()
    h['height_estimation_error_hist'][k]=np.abs(estimate[2,0]-truth[2,0]).max()
    reference=scenario.INITIAL_POSITIONS+truth[:,0,None]
    cbf=CBF(); previous=np.zeros((3,6)) if k==0 else h['u_safe_hist'][...,k-1]
    bound=h['u_max_hist'][:,k];du=cbf.accel_slew_rate*scenario.DT
    lb=np.maximum(-bound[None,:],previous-du);ub=np.minimum(bound[None,:],previous+du)
    free=np.clip((h['nominal_acceleration_hist'][...,k]+cbf.smoothing_weight*previous)/(1+cbf.smoothing_weight),lb,ub)
    E=np.eye(3)/(scenario.OBSTACLE_RADIUS+scenario.OBSTACLE_DESIGN_CLEARANCE)**2
    for o,center in enumerate(scenario.OBSTACLE_CENTERS.T):
        h['min_phys_obs_dist'][o,k]=min(ellipsoid_radial_clearance(p[:,i],center,np.full(3,scenario.OBSTACLE_RADIUS)) for i in range(6))
        h['reference_obstacle_clearance_hist'][o,k]=(np.linalg.norm(reference-center[:,None],axis=0)-scenario.OBSTACLE_RADIUS).min()
        for i in range(6):
            delta=p[:,i]-center;row=-2*delta@E;norm=max(np.linalg.norm(row),1e-10)
            rhs=2*v[:,i]@E@v[:,i]+2*(2*delta@E@v[:,i])+.96*(delta@E@delta-1)-2*np.linalg.norm(E@delta)*.4
            h['obstacle_qp_margin_hist'][o,i,k]=(rhs-row@h['u_safe_hist'][:,i,k])/norm
            h['obstacle_free_margin_hist'][o,i,k]=(rhs-row@free[:,i])/norm
    h['U_avg_hist'][...,k]=h['U_hist'][...,k].mean(axis=2)
    h['v_est_hist'][...,k]=estimate[:,1,:]
    obs=errors[1];plant=np.linalg.norm(v-h['flight_velocity_target_hist'][...,k],axis=0)
    cohesion=np.linalg.norm(v-v.mean(axis=1)[:,None],axis=0)
    for key,value in [('redcho_agent_err_hist',obs),('plant_agent_err_hist',plant),('cohesion_agent_err_hist',cohesion),
        ('redcho_observer_err_hist',obs.mean()),('plant_tracking_err_hist',plant.mean()),('vel_consensus_err_hist',cohesion.mean())]:h[key][...,k]=value
    h['svf_regularizer_hist'][k]=np.linalg.norm(h['svf_snap_hist'][...,k],axis=0).max()


def performance_summary(h,meta):
    if not len(h['time']): raise ValueError('No complete cycles to evaluate')
    airborne=h['p_hist'][2]>scenario.GROUND_COM_HEIGHT+.01
    consecutive=airborne.copy();consecutive[:,0]=False;consecutive[:,1:] &= airborne[:,:-1]
    residual=float(h['model_mismatch_hist'][consecutive].max()) if consecutive.any() else 0.
    active=(h['obstacle_qp_margin_hist']<=1e-5)&(h['obstacle_free_margin_hist']<-1e-6)
    s=dict(architecture=scenario.ARCHITECTURE,banks_per_agent=1,duration=meta['duration'],
        max_svf_derivative=float(h['svf_regularizer_hist'].max()),min_interagent_distance=float(h['min_dist_hist'].min()),
        min_obstacle_clearance=h['min_phys_obs_dist'].min(axis=1).tolist(),
        max_qp_violation=float(h['qp_violation_hist'].max()),max_phase1_slack=float(h['qp_phase1_slack_hist'].max()),
        phase1_invocations=int(np.count_nonzero(h['qp_mode_hist'])),agent_steps=int(h['qp_mode_hist'].size),
        max_freeflight_residual=residual, max_all_phase_residual=float(h['model_mismatch_hist'].max()),
        final_order_error=h['position_redcho_order_error_hist'][:,-1].tolist(),
        final_velocity_consensus_error=float(h['vel_consensus_err_hist'][-1]),
        position_regularity_max=float(np.linalg.norm(h['pose_regularity_hist'],axis=0).max()),
        recovery_evaluated=False)
    s['checks']=dict(finite=all(np.isfinite(a).all() for a in h.values()),
        svf_derivative=s['max_svf_derivative']<=meta['L_svf']*(1+1e-6),
        hard_qp_feasibility=max(s['max_qp_violation'],s['max_phase1_slack'])<=1e-6,
        plant_residual=residual<=.40+1e-6,
        interagent_safety=s['min_interagent_distance']>=scenario.INTERAGENT_DISTANCE-1e-3,
        obstacle_clearance=min(s['min_obstacle_clearance'])>=scenario.OBSTACLE_CLEARANCE-1e-3,
        position_REDCHO_regularity=s['position_regularity_max']<=scenario.POSE_REDCHO_L/2.)
    gate=int(round(scenario.FORWARD_START/scenario.DT))
    if len(h['time'])>gate:
        s['hover_gate_height_error']=float(np.abs(h['p_hist'][2,:,gate]-scenario.HOVER_HEIGHT).max())
        s['hover_gate_speed']=float(np.linalg.norm(h['v_hist'][...,gate],axis=0).max())
        s['checks']['hover_before_forward']=s['hover_gate_height_error']<=scenario.HOVER_HEIGHT_TOLERANCE and s['hover_gate_speed']<=scenario.HOVER_SPEED_TOLERANCE
    s['obstacle_encounters']=[dict(obstacle=o+1,active_agent_samples=int(active[o].sum()),
        nominal_reference_min_surface_clearance=float(h['reference_obstacle_clearance_hist'][o].min()),
        actual_min_surface_clearance=s['min_obstacle_clearance'][o]) for o in range(2)]
    if h['time'][-1]>=scenario.RECOVERY_CHECK_START+scenario.RECOVERY_WINDOW-1e-8:
        s['recovery_evaluated']=True;tail=h['time']>=h['time'][-1]-scenario.RECOVERY_WINDOW-1e-8
        keys=['formation_shape_error_hist','height_spread_hist','hover_height_error_hist','velocity_spread_hist']
        r={key:float(h[key][tail].max()) for key in keys};r['minimum_x']=float(h['p_hist'][0,:,tail].min())
        r['minimum_obstacle_clearance']=float(h['min_phys_obs_dist'][:,tail].min());s['recovery']=r
        s['checks'].update(formation_recovered=r[keys[0]]<=scenario.FORMATION_POSITION_TOLERANCE,
            common_height_recovered=r[keys[1]]<=scenario.FORMATION_HEIGHT_TOLERANCE,
            hover_height_recovered=r[keys[2]]<=scenario.FORMATION_HEIGHT_TOLERANCE,
            velocity_consensus_recovered=r[keys[3]]<=scenario.FORMATION_VELOCITY_TOLERANCE,
            default_scene_progress=r['minimum_x']>scenario.RECOVERY_PASS_X,
            recovery_window_obstacle_clear=r['minimum_obstacle_clearance']>=scenario.OBSTACLE_DESIGN_CLEARANCE,
            position_REDCHO_estimation=float(h['pose_redcho_agent_error_hist'][0,:,tail].max())<=scenario.POSE_ESTIMATION_TOLERANCE,
            height_REDCHO_estimation=float(h['height_estimation_error_hist'][tail].max())<=scenario.POSE_ESTIMATION_TOLERANCE,
            both_obstacles_in_nominal_path=all(e['nominal_reference_min_surface_clearance']<scenario.OBSTACLE_CLEARANCE for e in s['obstacle_encounters']),
            both_obstacles_require_QP_correction=all(e['active_agent_samples']>0 for e in s['obstacle_encounters']))
    return s


def export_csv(h,output):
    output=Path(output);agent_keys=[*AGENT_SHAPES,'p_hist','v_hist']
    columns=[]
    for key in agent_keys:
        shape=h[key].shape[:-2]
        for index in np.ndindex(shape):columns.append((key,index,key+('['+','.join(map(str,index))+']' if index else '')))
    with (output/'agent_telemetry.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s','agent_id',*[x[2] for x in columns]])
        for k,t in enumerate(h['time']):
            for i in range(6):writer.writerow([float(t),i+1,*[float(h[key][(*index,i,k)]) for key,index,_ in columns]])
    with (output/'global_telemetry.csv').open('w',newline='') as f:
        keys=[k for k,shape in GLOBAL_SHAPES.items() if not shape]
        writer=csv.writer(f);writer.writerow(['time_s',*keys])
        for k,t in enumerate(h['time']):writer.writerow([float(t),*[float(h[key][k]) for key in keys]])
    (output/'telemetry_schema.json').write_text(json.dumps(dict(
        architecture=scenario.ARCHITECTURE,redcho_orders=['position_m','velocity_mps','acceleration_mps2'],
        arrays={k:list(a.shape) for k,a in h.items()},matrix_axes='xyz, derivative_order, agent, time',
        agent_csv_columns=['time_s','agent_id',*[x[2] for x in columns]],
        note='No independent velocity or Levant REDCHO exists; absent arrays are not filled with zeros'),indent=2))

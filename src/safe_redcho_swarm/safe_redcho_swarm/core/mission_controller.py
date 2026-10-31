"""Unchanged baseline controller equations with an external plant adapter."""
from __future__ import annotations
from dataclasses import asdict
import json
from pathlib import Path
import time as walltime
import numpy as np
from scipy.integrate import quad
from safe_redcho_swarm.core.config import Params, Battery, CBF
from safe_redcho_swarm.core.battery import compute_vocv, speed_governor, voltage_aware_accel_bound
from safe_redcho_swarm.core.reference_filters import compute_svf_iss_certificate, svf_step, levant_step
from safe_redcho_swarm.core.redcho import build_redcho_matrices, redcho_state_derivative, redcho_strict_kernel
from safe_redcho_swarm.core.cbf_qp import solve_decentralized_hocbf_qp
from safe_redcho_swarm.core.quadrotor import (flatness_attitude_from_force, ellipsoid_radial_clearance,
                       crazyflie_geometric_controller)


def baseline_initial_state():
    params,batt = Params(),Battery()
    soc = np.array([1.,.95,.90,.65,.80,.75])
    speed = speed_governor(soc,batt)
    p = np.array([[-3.5-(i//2)*1.25,-3.5-(i%2)*1.25,.2]
                  for i in range(6)]).T
    v = np.vstack([speed,np.zeros((2,6))])
    R = np.stack([flatness_attitude_from_force(
        params.m*params.g*np.array([0.,0.,1.])+
        params.D_drag*np.linalg.norm(v[:,i])*v[:,i],params.max_tilt,0.)
        for i in range(6)],axis=2)
    return p,v,R,np.zeros((3,6))


class MissionController:
    """Same controller loop as baseline.py; plant states enter via send()."""
    def __init__(self,duration=40.,dt=.01,initial_state=None,progress=True):
        self.initial_state=initial_state
        self.h={}
        self.metadata={}
        self.completed=0
        self.stream=self._run(duration,dt,progress)

    def snapshot(self):
        return {key:(value[...,:self.completed].copy())
                for key,value in self.h.items()} | {
                    'time':self.time[:self.completed].copy()}

    def _run(self,duration,dt,progress):
        params, batt, cbf = Params(), Battery(), CBF(sample_time=dt)
        N, m1, fault_time, fault_id = 6, 3, 12., 1
        # Preserve the attached-code scenario.
        soc_initial = np.array([1., .95, .90, .65, .80, .75])
        p_obs = np.array([[0., 9.], [-1., -1.], [.2, .2]])
        r_obs = np.full((3, 2), .5)
        E_mats = [np.diag(1/(r_obs[:, o]+.75)**2) for o in range(2)]
        adjacency = np.zeros((N, N), dtype=int)
        for i in range(N): adjacency[i, (i-1) % N] = adjacency[i, (i+1) % N] = 1
        gamma, theta, gains, eps_sm, omega = np.full(3, .15), 1.2, np.array([2., 1.4, .8]), .04, 2.
        _, G = build_redcho_matrices(gamma)
        coeff = np.poly(-gamma)
        c_w = coeff[1:][::-1]-np.array([omega**3, 3*omega**2, 3*omega])
        K_w = quad(lambda tau: abs(redcho_strict_kernel(tau, omega, c_w)), 0, np.inf, epsabs=1e-12, epsrel=1e-10)[0]
        s0 = speed_governor(soc_initial, batt)
        L_ic = abs(c_w[0])*np.max(np.abs(s0-s0.mean()))
        L_R_star = L_ic+2*.65*(omega**3+K_w)
        L = np.ceil(100*1.10*L_R_star)/100
        L_svf = compute_svf_iss_certificate(omega, .65, .65)['L_svf_vector']
        print(f'Certificate values: L_svf={L_svf:.4f}, K_w={K_w:.4f}, L_ic={L_ic:.4f}, L_R*={L_R_star:.4f}, selected L={L:.2f}', flush=True)
        steps = int(round(duration/dt))+1
        if abs((steps-1)*dt-duration) > 1e-9: raise ValueError('duration must be an integer multiple of dt')
        time = np.arange(steps)*dt
        p = np.zeros((3, N)); v = np.zeros((3, N)); R = np.repeat(np.eye(3)[:, :, None], N, axis=2); Om = np.zeros((3, N))
        Z = np.zeros((3, m1, N)); Zlev = np.zeros_like(Z); X = np.zeros_like(Z); Xlev = np.zeros_like(Z)
        for i in range(N):
            p[:, i] = [-3.5-(i//2)*1.25, -3.5-(i % 2)*1.25, .2]
            v[:, i] = Z[:, 0, i] = Zlev[:, 0, i] = np.array([s0[i], 0., 0.])
            force = params.m*params.g*np.array([0., 0., 1.])+params.D_drag*np.linalg.norm(v[:, i])*v[:, i]
            R[:, :, i] = flatness_attitude_from_force(force, params.max_tilt, 0)
        soc, Vp, Vterm, Rint = soc_initial.copy(), np.zeros(N), compute_vocv(soc_initial), np.full(N, batt.R_int_nom)
        uprev, vprev = np.zeros((3, N)), np.zeros((3, N))
        h = {}
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
        self.h = h
        self.time = time
        self.completed = 0
        self.metadata = dict(params=asdict(params),battery=asdict(batt),cbf=asdict(cbf),
            duration=duration,dt=dt,fault_time=fault_time,fault_agent_id=fault_id+1,
            L_svf=L_svf,K_w=K_w,L_ic=L_ic,L_R_star=L_R_star,L=L,
            initial_soc=soc_initial.tolist(),obstacle_centers=p_obs.tolist(),
            obstacle_radii=r_obs.tolist(),
            solver='SciPy SLSQP; unchanged baseline QP objective and constraints',
            plant_backend='Gazebo Harmonic, native ten-step motor-force backend',
            hardware_target='Crazyflie 2.1+; MATLAB-derived surrogate parameters')
        if self.initial_state is not None:
            p,v,R,Om = (np.array(a,dtype=float,copy=True) for a in self.initial_state)
        self.initial_plant_state = tuple(a.copy() for a in (p,v,R,Om))
        fault_injected = False
        started = walltime.perf_counter()
        for k, t in enumerate(time):
            if t >= fault_time and not fault_injected:
                soc[fault_id], Rint[fault_id] = .32, 1.5*Rint[fault_id]
                fault_injected = True
            h['p_hist'][...,k], h['v_hist'][...,k] = p, v
            h['SoC_hist'][:,k], h['V_batt_hist'][:,k] = soc, Vterm
            h['v_mag_hist'][:,k] = np.linalg.norm(v, axis=0)
            measured = np.zeros_like(v) if k == 0 else (v-vprev)/dt
            mismatch = np.linalg.norm(measured-uprev, axis=0)
            h['model_mismatch_hist'][:,k], h['disturbance_bound_hist'][:,k] = mismatch, cbf.plant_mismatch_bound
            if k > 0 and cbf.strict_mismatch_bound and np.max(mismatch) > cbf.plant_mismatch_bound+1e-6:
                i = int(np.argmax(mismatch))
                raise RuntimeError(f'SE(3) acceleration residual exceeded bound at t={t:.3f}, agent {i+1}: {mismatch[i]:.6f} > {cbf.plant_mismatch_bound}')
            distances = np.linalg.norm(p[:,:,None]-p[:,None,:], axis=0)
            np.fill_diagonal(distances, np.inf)
            h['min_dist_hist'][k] = distances.min()
            for o in range(2): h['min_phys_obs_dist'][o,k] = min(ellipsoid_radial_clearance(p[:,i],p_obs[:,o],r_obs[:,o]) for i in range(N))
            heading = np.array([np.cos(.04*t), np.sin(.04*t), .05*np.sin(.2*t)])
            heading /= np.linalg.norm(heading)
            speeds = speed_governor(soc, batt)
            raw = heading[:,None]*speeds
            h['v_safe_hist'][:,k], h['raw_ref_hist'][...,k] = speeds, raw
            Z, snap = svf_step(Z, raw, omega, dt)
            h['svf_snap_hist'][...,k], h['svf_regularizer_hist'][k] = snap, np.max(np.linalg.norm(snap,axis=0))
            Zlev = levant_step(Zlev, raw, L, dt)
            Y = Z-np.einsum('ab,dbi->dai',G,X)
            Ylev = Zlev-np.einsum('ab,dbi->dai',G,Xlev)
            for name,val in [('U_hist',Z),('Y_hist',Y),('U_avg_hist',Z.mean(axis=2)),('U_levant_hist',Zlev),
                             ('Y_levant_hist',Ylev),('U_levant_avg_hist',Zlev.mean(axis=2)),('v_est_hist',Y[:,0,:])]: h[name][...,k]=val
            obs_err = np.linalg.norm(Y[:,0,:]-Z[:,0,:].mean(axis=1)[:,None],axis=0)
            plant_err = np.linalg.norm(v-Y[:,0,:],axis=0)
            cohesion_err = np.linalg.norm(v-v.mean(axis=1)[:,None],axis=0)
            for name,val in [('redcho_agent_err_hist',obs_err),('plant_agent_err_hist',plant_err),('cohesion_agent_err_hist',cohesion_err),
                             ('redcho_observer_err_hist',obs_err.mean()),('plant_tracking_err_hist',plant_err.mean()),('vel_consensus_err_hist',cohesion_err.mean())]: h[name][...,k]=val
            h['redcho_order_err_svf'][:,k] = np.linalg.norm(Y-Z.mean(axis=2)[:,:,None],axis=0).mean(axis=1)
            h['redcho_order_err_levant'][:,k] = np.linalg.norm(Ylev-Zlev.mean(axis=2)[:,:,None],axis=0).mean(axis=1)
            X += dt*redcho_state_derivative(X,Y,adjacency,gamma,theta,gains,L,eps_sm)
            Xlev += dt*redcho_state_derivative(Xlev,Ylev,adjacency,gamma,theta,gains,L,eps_sm)
            err = v-Y[:,0,:]
            unom = Y[:,1,:]-4.5*err-.45*np.tanh(err)
            bounds = np.array([voltage_aware_accel_bound(Vterm[i],params,batt,cbf.max_design_accel) for i in range(N)])
            Fmax, abox = bounds[:,0], bounds[:,1]
            h['F_max_hist'][:,k], h['u_max_hist'][:,k] = Fmax, abox
            usafe = np.zeros((3,N))
            info_map = dict(phase1_slack='qp_phase1_slack_hist',max_violation='qp_violation_hist',mode='qp_mode_hist',
                            row_margin='row_actuator_margin_hist',recursive_margin='recursive_feas_margin_hist')
            for i in range(N):
                usafe[:,i], info = solve_decentralized_hocbf_qp(i,unom[:,i],uprev,p,v,abox,np.full(N,cbf.plant_mismatch_bound),
                                                             .92,p_obs,E_mats,2.8,3.2,.8,1.2,cbf)
                for field,key in info_map.items(): h[key][i,k]=info[field]
                if cbf.strict_feasibility and info['phase1_slack'] > 10*cbf.qp_tolerance:
                    raise RuntimeError(f'Hard HOCBF-QP infeasible at t={t:.3f}, agent {i+1}, Phase-I slack {info["phase1_slack"]:.3e}')
            h['u_safe_hist'][...,k] = usafe
            h['qp_correction_hist'][:,k] = np.linalg.norm(usafe-unom,axis=0)
            h['accel_utilization_hist'][:,k] = np.max(np.abs(usafe),axis=0)/np.maximum(abox,1e-9)
            h['slew_utilization_hist'][:,k] = np.max(np.abs(usafe-uprev),axis=0)/max(cbf.accel_slew_rate*dt,1e-9)
            # Store pre-integration velocity; do not alias the mutable plant array.
            vprev = v.copy()
            for i in range(N):
                motors, att_err = crazyflie_geometric_controller(R[:,:,i],Om[:,i],v[:,i],usafe[:,i],params,Fmax[i])
                h['u_motors_hist'][:,i,k], h['attitude_error_hist'][i,k] = motors, att_err
                h['motor_utilization_hist'][i,k] = motors.max()/max(Fmax[i],1e-9)
                # Gazebo advances all agents atomically after this loop.
                power = params.c_prop*motors.sum()**1.5+params.P_avionics
                current = power/max(Vterm[i],2.5)
                h['P_elec_hist'][i,k],h['I_batt_hist'][i,k] = power,current
                soc[i] = np.clip(soc[i]-dt*current/(3600*batt.Q_nom),.01,1.)
                Vp[i] += dt*(-Vp[i]/(batt.R_p*batt.C_p)+current/batt.C_p)
                Vterm[i] = max(2.5,compute_vocv(soc[i])-Vp[i]-current*Rint[i])
            uprev = usafe.copy()
            self.completed = k+1
            feedback = yield dict(time=float(t), motors=h['u_motors_hist'][...,k].copy(),
                                  limits=Fmax.copy(), accel=usafe.copy(),
                                  state=tuple(a.copy() for a in (p,v,R,Om)))
            p,v,R,Om = (np.array(a,dtype=float,copy=True) for a in feedback)
            if progress and (k % max(1,int(round(5/dt))) == 0 or k==steps-1):
                print(f't={t:5.2f}/{duration:.2f} s; min distance={h["min_dist_hist"][k]:.4f} m; elapsed={walltime.perf_counter()-started:.1f} s',flush=True)
        h['time'] = time
        self.metadata['final_rotation_orthogonality_error'] = float(
            max(np.linalg.norm(R[:,:,i].T@R[:,:,i]-np.eye(3)) for i in range(N)))
        self.metadata['state_log_end_time'] = float(time[-1])
        self.metadata['plant_advance_duration'] = float(steps*dt)
        metadata = self.metadata
        return h, metadata

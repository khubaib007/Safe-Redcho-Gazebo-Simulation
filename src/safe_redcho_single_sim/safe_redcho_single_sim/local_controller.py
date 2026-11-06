"""One local battery, one position REDCHO, unchanged HOCBF-QP and motor law.

Derived from safe_redcho_distributed/local_controller.py; only reference-bank
architecture and its associated telemetry are changed.
"""
import numpy as np
from safe_redcho_swarm.core.config import Params,Battery,CBF
from safe_redcho_swarm.core.battery import compute_vocv,speed_governor,voltage_aware_accel_bound
from safe_redcho_swarm.core.cbf_qp import solve_decentralized_hocbf_qp
from safe_redcho_swarm.core.quadrotor import crazyflie_geometric_controller,flatness_attitude_from_force
from .reference import SinglePositionReference
from .formation_control import apply_formation,validate_configuration

from . import scenario

INITIAL_SOC=np.array([1.,.95,.90,.65,.80,.75])


def initial_plant_state():
    params,batt=Params(),Battery()
    return scenario.INITIAL_POSITIONS.copy(),np.zeros((3,6)),np.repeat(np.eye(3)[:,:,None],6,axis=2),np.zeros((3,6))


class LocalController:
    def __init__(self,agent_id):
        validate_configuration()
        self.agent_id=agent_id;self.i=agent_id-1
        self.reference=SinglePositionReference(agent_id);self.neighbors=self.reference.neighbors
        self.params=Params();self.batt=Battery();self.cbf=CBF(sample_time=.01);self.dt=.01
        self.soc=INITIAL_SOC[self.i];self.Vp=0.;self.voltage=compute_vocv(INITIAL_SOC)[self.i]
        self.resistance=self.batt.R_int_nom;self.faulted=False
        self.vprev=np.zeros(3);self.previous_accel=np.zeros(3);self.previous_freeflight=False
        self.obstacles=scenario.OBSTACLE_CENTERS.copy()
        self.E=[np.diag(1/(np.full(3,scenario.OBSTACLE_RADIUS)+scenario.OBSTACLE_DESIGN_CLEARANCE)**2) for _ in range(2)]
        self.finished=True

    def prepare(self,cycle,t,state,previous):
        if not self.finished:raise ValueError('Previous local control cycle incomplete')
        if abs(t-(cycle-1)*self.dt)>1e-9:raise ValueError('Control cycle/time mismatch')
        p,v,R,w=(np.array(x,dtype=float,copy=True) for x in state)
        for value,shape in zip((p,v,R,w),((3,6),(3,6),(3,3,6),(3,6))):
            if value.shape!=shape or not np.isfinite(value).all():raise ValueError('Invalid physical state batch')
        previous=np.array(previous,dtype=float,copy=True)
        if previous.shape!=(3,6) or not np.isfinite(previous).all():raise ValueError('Invalid previous acceleration batch')
        if not np.allclose(previous[:,self.i],self.previous_accel,rtol=0,atol=1e-7):raise ValueError('Own previous acceleration disagrees with coordinator')
        if t>=scenario.FORWARD_START+12. and self.i==1 and not self.faulted:
            self.soc=.32;self.resistance*=1.5;self.faulted=True
        measured=np.zeros(3) if cycle==1 else (v[:,self.i]-self.vprev)/self.dt
        mismatch=float(np.linalg.norm(measured-previous[:,self.i]))
        self.current_freeflight=p[2,self.i]>scenario.GROUND_COM_HEIGHT+.01
        if cycle>1 and self.previous_freeflight and self.current_freeflight and self.cbf.strict_mismatch_bound and mismatch>self.cbf.plant_mismatch_bound+1e-6:
            raise RuntimeError(f'cf{self.agent_id} residual {mismatch} exceeds .40 at t={t}')
        if cycle==int(round(scenario.FORWARD_START/self.dt))+1:
            height_error=np.max(np.abs(p[2]-scenario.HOVER_HEIGHT))
            max_speed=np.max(np.linalg.norm(v,axis=0))
            if height_error>scenario.HOVER_HEIGHT_TOLERANCE or max_speed>scenario.HOVER_SPEED_TOLERANCE:
                raise RuntimeError(f'Hover gate failed: height error={height_error:.4f}m, speed={max_speed:.4f}m/s')
        self.time=t
        speed=speed_governor(np.array([self.soc]),self.batt)[0]
        if t<scenario.FORWARD_START:
            raw=np.zeros(3)
        else:
            tau=t-scenario.FORWARD_START
            heading=np.array([np.cos(.04*tau),np.sin(.04*tau),.05*np.sin(.2*tau)])
            heading/=np.linalg.norm(heading)
            raw=heading*speed
        Fmax,abox=voltage_aware_accel_bound(self.voltage,self.params,self.batt,self.cbf.max_design_accel)
        outgoing=self.reference.prepare(cycle,t,raw)
        self.state=(p,v,R,w);self.previous=previous;self.limit=Fmax;self.bound=abox;self.finished=False
        self.diagnostics={'SoC_hist':float(self.soc),'V_batt_hist':float(self.voltage),'v_safe_hist':float(speed),
                          'raw_ref_hist':raw,'F_max_hist':float(Fmax),'u_max_hist':float(abox),
                          'model_mismatch_hist':mismatch,'disturbance_bound_hist':self.cbf.plant_mismatch_bound}
        return outgoing,abox

    def finish(self,peers,bounds):
        if self.finished:raise ValueError('Duplicate control completion')
        bounds=np.asarray(bounds,dtype=float)
        if bounds.shape!=(6,) or not np.isfinite(bounds).all() or np.any(bounds<=0):raise ValueError('Invalid same-cycle safety bounds')
        if abs(bounds[self.i]-self.bound)>1e-12:raise ValueError('Own safety bound disagrees with batch')
        obs=self.reference.finish(peers);p,v,R,w=self.state
        nominal,formation_diagnostics=apply_formation(
            self.agent_id,self.time,obs['pose_redcho_estimate_hist'],p[:,self.i],v[:,self.i])
        tracking_target=formation_diagnostics['flight_velocity_target_hist']
        safe,info=solve_decentralized_hocbf_qp(self.i,nominal,self.previous,p,v,bounds,np.full(6,.4),
                                             scenario.INTERAGENT_DESIGN_DISTANCE,self.obstacles,self.E,2.8,3.2,.8,1.2,self.cbf)
        if (not np.isfinite(safe).all() or not np.isfinite([info['phase1_slack'],info['max_violation']]).all()
                or (self.cbf.strict_feasibility and max(info['phase1_slack'],info['max_violation'])>10*self.cbf.qp_tolerance)):
            raise RuntimeError(f'cf{self.agent_id} hard QP infeasible at t={self.time:.2f}: {info}')
        motors,attitude=crazyflie_geometric_controller(R[:,:,self.i],w[:,self.i],v[:,self.i],safe,self.params,self.limit)
        power=self.params.c_prop*motors.sum()**1.5+self.params.P_avionics
        current=power/max(self.voltage,2.5)
        d=self.diagnostics
        d.update(formation_diagnostics)
        d.update(obs)
        d['nominal_acceleration_hist']=nominal.copy()
        d['flight_velocity_target_hist']=tracking_target
        d.update(u_safe_hist=safe,u_motors_hist=motors,P_elec_hist=float(power),I_batt_hist=float(current),
                 attitude_error_hist=float(attitude),motor_utilization_hist=float(motors.max()/max(self.limit,1e-9)),
                 accel_utilization_hist=float(np.max(np.abs(safe))/max(self.bound,1e-9)),
                 slew_utilization_hist=float(np.max(np.abs(safe-self.previous[:,self.i]))/(8*self.dt)),
                 qp_correction_hist=float(np.linalg.norm(safe-nominal)),plant_agent_err_hist=float(np.linalg.norm(v[:,self.i]-tracking_target)))
        for field,key in {'phase1_slack':'qp_phase1_slack_hist','max_violation':'qp_violation_hist','mode':'qp_mode_hist',
                          'row_margin':'row_actuator_margin_hist','recursive_margin':'recursive_feas_margin_hist'}.items():d[key]=info[field]
        self.soc=np.clip(self.soc-self.dt*current/(3600*self.batt.Q_nom),.01,1.)
        self.Vp+=self.dt*(-self.Vp/(self.batt.R_p*self.batt.C_p)+current/self.batt.C_p)
        self.voltage=max(2.5,compute_vocv(self.soc)-self.Vp-current*self.resistance)
        self.vprev=v[:,self.i].copy();self.previous_accel=safe.copy();self.previous_freeflight=self.current_freeflight;self.finished=True
        return d

"""Same nominal controller, with p/v/a supplied by one position REDCHO bank."""
import numpy as np
from . import scenario
from .pose_redcho import regularity_certificate


def validate_configuration():
    p = scenario.INITIAL_POSITIONS
    if p.shape != (3,6) or not np.isfinite(p).all() or np.ptp(p[2]) > 1e-10:
        raise ValueError('Require six finite equal-height initial positions')
    d = np.linalg.norm(p[:,:,None]-p[:,None,:], axis=0); np.fill_diagonal(d, np.inf)
    if d.min() <= scenario.INTERAGENT_DESIGN_DISTANCE:
        raise ValueError('Initial spacing violates buffered safety distance')
    if not 0 <= scenario.TAKEOFF_START < scenario.TAKEOFF_END < scenario.FORWARD_START:
        raise ValueError('Invalid ground/takeoff/hover ordering')
    for key in ('FORMATION_POSITION_GAIN','FORMATION_POSITION_ACCEL_CAP'):
        a = np.asarray(getattr(scenario,key))
        if a.shape != (3,) or not np.isfinite(a).all() or np.any(a <= 0):
            raise ValueError('Invalid '+key)
    if regularity_certificate()['disagreement_bound'] >= scenario.POSE_REDCHO_L:
        raise ValueError('Position REDCHO regularity budget insufficient')


def smooth_activation(t):
    s = np.clip((t-scenario.FORWARD_START)/scenario.FORMATION_RAMP_TIME, 0., 1.)
    return float(s**3*(10.-15.*s+6.*s*s))


def apply_formation(agent_id, t, position_redcho, position, velocity):
    pr = np.asarray(position_redcho,float)
    if pr.shape != (3,3) or not np.isfinite(pr).all():
        raise ValueError('Require one finite position/velocity/acceleration REDCHO output')
    target = scenario.INITIAL_POSITIONS[:,agent_id-1]+pr[:,0]
    error = target-position
    cap = scenario.FORMATION_POSITION_ACCEL_CAP
    correction = cap*np.tanh(scenario.FORMATION_POSITION_GAIN*error/cap)
    activation = smooth_activation(t)
    desired_velocity, feedforward = pr[:,1].copy(), pr[:,2].copy()
    ev = velocity-desired_velocity
    nominal = feedforward-4.5*ev-.45*np.tanh(ev)
    nominal[:2] += activation*correction[:2]
    nominal[2] += correction[2]
    if t < scenario.FORWARD_START:
        nominal = feedforward+6.*error+4.*(desired_velocity-velocity)
    return nominal, dict(formation_correction_hist=np.r_[activation*correction[:2],correction[2]],
                         formation_position_error_hist=-error,
                         formation_velocity_error_hist=ev, formation_activation_hist=activation,
                         formation_target_hist=target, flight_velocity_target_hist=desired_velocity)

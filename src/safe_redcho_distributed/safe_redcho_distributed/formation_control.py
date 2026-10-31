"""Track distributed REDCHO position/height references before the hard QP."""
import numpy as np
from . import scenario
from .pose_redcho import regularity_certificate


def validate_configuration():
    positions = np.asarray(scenario.INITIAL_POSITIONS, float)
    if positions.shape != (3, 6) or not np.isfinite(positions).all():
        raise ValueError('Expected finite 3 x 6 initial positions')
    if np.ptp(positions[2]) > 1e-10:
        raise ValueError('Common-height formation requires equal initial COM heights')
    distances = np.linalg.norm(positions[:, :, None]-positions[:, None, :], axis=0)
    np.fill_diagonal(distances, np.inf)
    if distances.min() <= scenario.INTERAGENT_DESIGN_DISTANCE:
        raise ValueError('Initial spacing must exceed buffered safety distance')
    for name in ('FORMATION_POSITION_GAIN', 'FORMATION_POSITION_ACCEL_CAP'):
        value = np.asarray(getattr(scenario, name), float)
        if value.shape != (3,) or not np.isfinite(value).all() or np.any(value <= 0):
            raise ValueError(name+' must contain three positive finite values')
    height = np.asarray(scenario.HEIGHT_PROPOSALS, float)
    if height.shape != (6,) or not np.isfinite(height).all() or np.any(height <= scenario.GROUND_COM_HEIGHT):
        raise ValueError('Six finite airborne height proposals are required')
    for name in ('FORMATION_RAMP_TIME','POSE_REDCHO_L','POSE_REDCHO_THETA','POSE_REDCHO_EPS','POSE_REFERENCE_BOUND'):
        value = float(getattr(scenario,name))
        if not np.isfinite(value) or value <= 0:
            raise ValueError(name+' must be positive and finite')
    if not (0. <= scenario.TAKEOFF_START < scenario.TAKEOFF_END < scenario.FORWARD_START):
        raise ValueError('Expected ground hold, takeoff and hover before forward flight')
    if regularity_certificate()['disagreement_bound'] >= scenario.POSE_REDCHO_L:
        raise ValueError('Position REDCHO input-regularity budget is insufficient')


def smooth_activation(t):
    s = np.clip((t-scenario.FORWARD_START)/scenario.FORMATION_RAMP_TIME, 0., 1.)
    return float(s**3*(10.-15.*s+6.*s*s))


def apply_formation(agent_id, t, velocity_redcho, pose_redcho, position, velocity):
    vr, pr = np.asarray(velocity_redcho, float), np.asarray(pose_redcho, float)
    if vr.shape != (3, 3) or pr.shape != (3, 3) or not np.isfinite(np.r_[vr.ravel(), pr.ravel()]).all():
        raise ValueError('Invalid REDCHO reference outputs')
    target = scenario.INITIAL_POSITIONS[:, agent_id-1]+pr[:, 0]
    error = target-position
    cap = scenario.FORMATION_POSITION_ACCEL_CAP
    correction = cap*np.tanh(scenario.FORMATION_POSITION_GAIN*error/cap)
    activation = smooth_activation(t)
    desired_velocity, feedforward = vr[:, 0].copy(), vr[:, 1].copy()
    desired_velocity[2], feedforward[2] = pr[2, 1], pr[2, 2]
    ev = velocity-desired_velocity
    nominal = feedforward-4.5*ev-.45*np.tanh(ev)
    nominal[:2] += activation*correction[:2]
    nominal[2] += correction[2]
    if t < scenario.FORWARD_START:
        desired_velocity = pr[:, 1].copy()
        nominal = pr[:, 2]+6.*error+4.*(desired_velocity-velocity)
    return nominal, dict(formation_correction_hist=np.r_[activation*correction[:2], correction[2]],
                         formation_position_error_hist=-error,
                         formation_velocity_error_hist=velocity-desired_velocity,
                         formation_activation_hist=activation,
                         formation_target_hist=target,
                         flight_velocity_target_hist=desired_velocity)

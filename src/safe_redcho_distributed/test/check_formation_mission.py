"""Closed-loop numerical SE(3) validation; this does not run native Gazebo."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed.local_controller import LocalController, initial_plant_state
from safe_redcho_distributed import scenario
from safe_redcho_distributed.mission_history import (
    allocate_history, record_global_diagnostics, mission_metadata, scenario_performance_summary,
)
from safe_redcho_swarm.core.config import Params
from safe_redcho_swarm.core.quadrotor import integrate_crazyflie_se3


def run(output, case='nominal', duration=45.):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output/'failure.json').unlink(missing_ok=True)
    if case == 'moved-obstacle':
        scenario.OBSTACLE_CENTERS[:, 0] += np.array([.35, .3, 0.])
    h = allocate_history(duration)
    agents = {i: LocalController(i) for i in range(1, 7)}
    state, previous, params = initial_plant_state(), np.zeros((3, 6)), Params()
    state[0][2] -= 9.81e-6
    state[1][2] -= .00981
    try:
        for k, t in enumerate(h['time']):
            prepared = {i: a.prepare(k+1, float(t), state, previous) for i, a in agents.items()}
            packets = {i: a.formation_packet() for i, a in agents.items()}
            bounds = np.array([prepared[i][1] for i in range(1, 7)])
            for i in range(6, 0, -1):
                a = agents[i]
                d = a.finish({j: prepared[j][0] for j in a.neighbors}, bounds,
                             {j: packets[j] for j in a.neighbors})
                for key, value in d.items():
                    h[key][..., i-1, k] = value
            record_global_diagnostics(h, k, state)
            if k % 500 == 0:
                print(f't={t:.1f}, pair={h["min_dist_hist"][k]:.4f}, '
                      f'shape={h["formation_shape_error_hist"][k]:.4f}, '
                      f'z_spread={h["height_spread_hist"][k]:.4f}, x_min={state[0][0].min():.3f}', flush=True)
            p, v, R, w = state
            for i in range(6):
                p[:, i], v[:, i], R[:, :, i], w[:, i] = integrate_crazyflie_se3(
                    p[:, i], v[:, i], R[:, :, i], w[:, i], h['u_motors_hist'][:, i, k], params, .01)
            if case == 'disturbance' and 28. <= t < 30.:
                # A bounded external acceleration, rather than a state teleport.
                # cf3/cf5 are separated and receive different perturbations.
                v[:, 2] += .01*np.array([.12, -.08, .08])
                v[:, 4] += .01*np.array([-.10, .06, -.06])
            contact = p[2] < .017425
            p[2, contact] = .017425
            v[2, contact] = np.maximum(v[2, contact], 0.)
            previous = h['u_safe_hist'][..., k].copy()
    except Exception as exc:
        np.savez_compressed(output/'partial_simulation_data.npz', **{key: a[..., :k] for key, a in h.items()})
        (output/'failure.json').write_text(json.dumps(dict(time=float(t), case=case, error=str(exc)), indent=2))
        raise
    meta = mission_metadata(duration)
    meta['validation_backend'] = 'Numerical SE(3) midpoint plant with simple floor proxy; no ROS/Gazebo execution'
    summary = scenario_performance_summary(h, meta)
    tail = h['time'] >= duration-scenario.RECOVERY_WINDOW
    recovery = {key: float(h[key][tail].max()) for key in [
        'formation_shape_error_hist', 'height_spread_hist', 'hover_height_error_hist',
        'x_alignment_error_hist', 'lateral_spacing_error_hist', 'velocity_spread_hist']}
    recovery['minimum_tail_x'] = float(h['p_hist'][0, :, tail].min())
    recovery['final_positions'] = h['p_hist'][..., -1].tolist()
    recovery['case'] = case
    summary['numerical_recovery'] = recovery
    np.savez_compressed(output/'simulation_data.npz', **h)
    (output/'performance_summary.json').write_text(json.dumps(summary, indent=2))
    (output/'metadata.json').write_text(json.dumps(meta, indent=2))
    print(json.dumps(recovery, indent=2), flush=True)
    assert all(summary['checks'].values()), summary['checks']
    assert np.isfinite(np.concatenate([a.ravel() for a in h.values()])).all()
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('formation_validation'))
    parser.add_argument('--case', choices=['nominal', 'disturbance', 'moved-obstacle'], default='nominal')
    args = parser.parse_args()
    run(args.output, args.case)

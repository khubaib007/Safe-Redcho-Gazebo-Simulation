"""Independent vectorized reference vs six ring-only position/height observers."""
import sys
from pathlib import Path
import json
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed.pose_redcho import PoseREDCHO, regularity_certificate
from safe_redcho_distributed import scenario
from safe_redcho_swarm.core.redcho import build_redcho_matrices, redcho_state_derivative
from safe_redcho_swarm.core.reference_filters import svf_step


def run(output=None):
    agents = {i: PoseREDCHO(i) for i in range(1, 7)}
    adjacency = np.zeros((6, 6), int)
    for i in range(6):
        adjacency[i, (i-1) % 6] = adjacency[i, (i+1) % 6] = 1
    gamma = np.full(3, .15)
    _, G = build_redcho_matrices(gamma)
    X, Z, velocity_svf = np.zeros((3, 3, 6)), np.zeros((3, 3, 6)), np.zeros((3, 3, 6))
    previous, maximum = np.zeros((2, 6)), 0.
    tail_error, height_estimates = [], []
    for k in range(4001):
        t = k*.01
        raw = np.zeros((3, 6))
        if t >= 5.:
            raw[0] = np.linspace(.35, .60, 6)
            if t >= 17.:
                raw[0, 1] = .25
        velocity_svf, _ = svf_step(velocity_svf, raw, 2., .01)
        messages = {i: a.prepare(k+1, t, velocity_svf[:, :, i-1]) for i, a in agents.items()}
        if k:
            Z[:2, 0] += .005*(previous+velocity_svf[:2, 0])
        previous = velocity_svf[:2, 0].copy()
        Z[:2, 1], Z[:2, 2] = velocity_svf[:2, 0], velocity_svf[:2, 1]
        duration = scenario.TAKEOFF_END-scenario.TAKEOFF_START
        s = np.clip((t-scenario.TAKEOFF_START)/duration, 0., 1.)
        dh = scenario.HEIGHT_PROPOSALS-scenario.GROUND_COM_HEIGHT
        Z[2, 0] = dh*(10*s**3-15*s**4+6*s**5)
        Z[2, 1] = dh*(30*s**2-60*s**3+30*s**4)/duration
        Z[2, 2] = dh*(60*s-180*s**2+120*s**3)/duration**2
        Y = Z-np.einsum('ab,dbi->dai', G, X)
        X += .01*redcho_state_derivative(X, Y, adjacency, gamma, 1.2,
                                        np.array([2., 1.4, .8]), scenario.POSE_REDCHO_L, .04)
        for i in range(6, 0, -1):
            a = agents[i]
            result = a.finish({j: messages[j] for j in a.neighbors})
            maximum = max(maximum, float(np.abs(result['pose_redcho_estimate_hist']-Y[:, :, i-1]).max()),
                          float(np.abs(a.X-X[:, :, i-1]).max()))
        # A true average is evaluated here only, as an independent test oracle.
        if t >= 35.:
            tail_error.append(float(np.linalg.norm(Y[:, 0]-Z[:, 0].mean(axis=1)[:, None], axis=0).max()))
        height_estimates = (scenario.GROUND_COM_HEIGHT+Y[2, 0]).tolist()
    assert maximum < 1e-12
    assert max(tail_error) < .01
    assert np.max(np.abs(np.asarray(height_estimates)-scenario.HEIGHT_PROPOSALS.mean())) < .001
    assert abs(height_estimates[0]-scenario.HEIGHT_PROPOSALS[0]) > .015
    a = PoseREDCHO(1)
    a.prepare(1, 0., np.zeros((3, 3)))
    try:
        a.finish({2: np.zeros(3)})
    except ValueError:
        pass
    else:
        raise AssertionError('Missing position REDCHO neighbor accepted')
    report = dict(status='PASS',cycles=4001,max_vectorized_difference=maximum,
                  max_tail_average_estimation_error=max(tail_error),
                  distinct_local_height_proposals=scenario.HEIGHT_PROPOSALS.tolist(),
                  final_distributed_height_estimates=height_estimates,
                  regularity_certificate=regularity_certificate(),
                  scope='Actual REDCHO states and ring exchange; independent vectorized oracle, no native ROS claim')
    if output:
        Path(output).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    run(sys.argv[1] if len(sys.argv) > 1 else None)

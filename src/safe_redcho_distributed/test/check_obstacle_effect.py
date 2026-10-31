"""Offline ablation only: is nearby obstacle information necessary?

All other controller blocks and pair/actuator constraints remain. There is no
ROS safety-disable parameter. This test succeeds by observing the missing
obstacle constraint cause a clearance violation against the actual geometry.
"""
import json
import argparse
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed.local_controller import LocalController, initial_plant_state
from safe_redcho_distributed import scenario
from safe_redcho_swarm.core.config import Params
from safe_redcho_swarm.core.quadrotor import integrate_crazyflie_se3


def run(output=None, obstacle=1):
    agents = {i: LocalController(i) for i in range(1, 7)}
    for a in agents.values():
        a.obstacles[:, obstacle-1] = [1000., 1000., 1000.]
    state, previous, params = initial_plant_state(), np.zeros((3, 6)), Params()
    state[0][2] -= 9.81e-6
    state[1][2] -= .00981
    for k in range(4501):
        t = k*.01
        prepared = {i: a.prepare(k+1, t, state, previous) for i, a in agents.items()}
        packets = {i: a.formation_packet() for i, a in agents.items()}
        bounds = np.array([prepared[i][1] for i in range(1, 7)])
        results = {i: a.finish({j: prepared[j][0] for j in a.neighbors}, bounds,
                               {j: packets[j] for j in a.neighbors}) for i, a in agents.items()}
        p, v, R, w = state
        clearances = np.linalg.norm(p[:, :, None]-scenario.OBSTACLE_CENTERS[:, None, :], axis=0)-scenario.OBSTACLE_RADIUS
        if clearances[:, obstacle-1].min() < scenario.OBSTACLE_CLEARANCE:
            i, o = int(np.argmin(clearances[:, obstacle-1])), obstacle-1
            report = dict(status='PASS', backend='Numerical plant; offline ablation only',
                          meaning='Without nearby obstacle constraints the formation controller violates required clearance',
                          time=t, agent=int(i+1), obstacle=int(o+1),
                          surface_clearance=float(clearances[i,o]), required=scenario.OBSTACLE_CLEARANCE)
            if output:
                Path(output).write_text(json.dumps(report, indent=2))
            print(json.dumps(report, indent=2))
            return
        for i in range(6):
            p[:, i], v[:, i], R[:, :, i], w[:, i] = integrate_crazyflie_se3(
                p[:, i], v[:, i], R[:, :, i], w[:, i], results[i+1]['u_motors_hist'], params, .01)
        contact = p[2] < .017425
        p[2, contact] = .017425
        v[2, contact] = np.maximum(v[2, contact], 0.)
        previous = np.column_stack([results[i]['u_safe_hist'] for i in range(1, 7)])
    raise AssertionError('Scenario did not demonstrate the obstacle constraint requirement')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--obstacle',type=int,choices=[1,2],required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    run(args.output,args.obstacle)

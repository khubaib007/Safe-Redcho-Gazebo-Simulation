"""Fault injection: an invalid solver result must never reach motor allocation."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed import local_controller as controller


def run():
    real_solver = controller.solve_decentralized_hocbf_qp
    try:
        for corruption in ('violation_without_slack', 'nonfinite_command', 'nonfinite_slack'):
            agent = controller.LocalController(1)
            state = controller.initial_plant_state()
            agent.prepare(1, 0., state, np.zeros((3, 6)))
            peers = {j: (np.zeros(3), np.zeros(3)) for j in agent.neighbors}
            formation = {j: np.zeros(3) for j in agent.neighbors}
            soc = agent.soc
            def corrupted_solver(*args):
                safe, info = real_solver(*args)
                if corruption == 'violation_without_slack':
                    info['phase1_slack'], info['max_violation'] = 0., .001
                elif corruption == 'nonfinite_command':
                    safe[0] = np.nan
                else:
                    info['phase1_slack'] = np.inf
                return safe, info
            controller.solve_decentralized_hocbf_qp = corrupted_solver
            try:
                agent.finish(peers, np.full(6, agent.bound), formation)
            except RuntimeError as exc:
                assert 'hard QP infeasible' in str(exc) and agent.soc == soc
            else:
                raise AssertionError('Invalid solver output accepted: '+corruption)
    finally:
        controller.solve_decentralized_hocbf_qp = real_solver
    print('PASS: unsafe hard violation, nonfinite acceleration and nonfinite slack rejected before motor/battery update')


if __name__ == '__main__':
    run()

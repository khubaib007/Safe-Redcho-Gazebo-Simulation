"""Verify REDCHO outputs actually influence position, height and velocity control."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed.formation_control import apply_formation, validate_configuration
from safe_redcho_distributed import scenario


def run():
    validate_configuration()
    p = scenario.INITIAL_POSITIONS[:, 0].copy()
    p[2] = scenario.HOVER_HEIGHT
    velocity, pose = np.zeros((3, 3)), np.zeros((3, 3))
    pose[2, 0] = scenario.HOVER_HEIGHT-scenario.GROUND_COM_HEIGHT
    baseline, _ = apply_formation(1, 10., velocity, pose, p, np.zeros(3))
    assert np.abs(baseline).max() < 1e-14
    shifted = pose.copy();shifted[0, 0] = .2
    nominal, _ = apply_formation(1, 10., velocity, shifted, p, np.zeros(3))
    assert nominal[0] > .1
    shifted = pose.copy();shifted[2, 0] += .2
    nominal, _ = apply_formation(1, 10., velocity, shifted, p, np.zeros(3))
    assert nominal[2] > .1
    shifted = pose.copy();shifted[2, 2] = .1
    nominal, _ = apply_formation(1, 10., velocity, shifted, p, np.zeros(3))
    assert abs(nominal[2]-.1) < 1e-14
    shifted_velocity = velocity.copy();shifted_velocity[0, 0] = .1
    nominal, _ = apply_formation(1, 10., shifted_velocity, pose, p, np.zeros(3))
    assert nominal[0] > .1
    # The position contribution itself remains bounded even for a large lag.
    shifted = pose.copy();shifted[0, 0] = 1000.
    _, diagnostics = apply_formation(1, 10., velocity, shifted, p, np.zeros(3))
    assert np.all(np.abs(diagnostics['formation_correction_hist']) <= scenario.FORMATION_POSITION_ACCEL_CAP)
    print('PASS: position/height REDCHO estimates and derivative feedforward affect commands; '
          'velocity REDCHO remains active; position feedback is bounded')


if __name__ == '__main__':
    run()

"""Order-2 distributed REDCHO for a 3D displacement reference (including z).

Local x/y reference integrates the existing SVF velocity output. Its exact
continuous-time derivative chain is (integral z_v0, z_v0, z_v1, z_v2).
Local z is an analytic quintic takeoff towards this agent's height proposal.
Only the three zeroth-order estimates are exchanged with the two ring peers.
"""
import numpy as np
from scipy.integrate import quad
from safe_redcho_swarm.core.redcho import build_redcho_matrices, redcho_state_derivative
from .ring_protocol import neighbors
from . import scenario


def regularity_certificate():
    """Conservative continuous-time bound for the reference generator.

    Uses the original positive third-order SVF impulse kernel and L1 norms of
    its first two derivatives. Valid while ||local displacement|| <= B0; the
    runtime checks enforce that finite operating horizon. This is an input
    regularity budget, not a theorem for the sampled closed-loop controller.
    """
    omega, vmax = 2., .65
    gain1 = 4.*np.exp(-2.)
    gain2 = quad(lambda x: np.exp(-x)*abs(1.-2.*x+.5*x*x), 0., np.inf,
                 epsabs=1e-10, epsrel=1e-10)[0]
    duration = scenario.TAKEOFF_END-scenario.TAKEOFF_START
    dh = float(np.max(scenario.HEIGHT_PROPOSALS)-scenario.GROUND_COM_HEIGHT)
    b1 = np.hypot(vmax, 1.875*dh/duration)
    b2 = np.hypot(omega*vmax*gain1, (10./np.sqrt(3.))*dh/duration**2)
    b3 = np.hypot(omega**2*vmax*gain2, 60.*dh/duration**3)
    bound = .15**3*scenario.POSE_REFERENCE_BOUND+3.*.15**2*b1+3.*.15*b2+b3
    return dict(reference_position_bound=scenario.POSE_REFERENCE_BOUND,
                first_derivative_bound=float(b1),second_derivative_bound=float(b2),
                third_derivative_bound=float(b3),local_regularity_bound=float(bound),
                disagreement_bound=float(2.*bound),selected_L=scenario.POSE_REDCHO_L,
                scope='Continuous reference-generator bound; sampled/smoothed REDCHO requires numerical validation')


def height_proposal(t, agent_id):
    duration = scenario.TAKEOFF_END-scenario.TAKEOFF_START
    s = np.clip((t-scenario.TAKEOFF_START)/duration, 0., 1.)
    dh = scenario.HEIGHT_PROPOSALS[agent_id-1]-scenario.GROUND_COM_HEIGHT
    value = dh*(10*s**3-15*s**4+6*s**5)
    velocity = dh*(30*s**2-60*s**3+30*s**4)/duration
    acceleration = dh*(60*s-180*s**2+120*s**3)/duration**2
    jerk = dh*(60-360*s+360*s**2)/duration**3 if 0. < s < 1. else 0.
    return value, velocity, acceleration, jerk


class PoseREDCHO:
    def __init__(self, agent_id):
        self.agent_id, self.index = agent_id, agent_id-1
        self.neighbors = neighbors(agent_id)
        self.dt = scenario.DT
        self.gamma = np.full(3, .15)
        self.gains = np.array([2., 1.4, .8])
        self.theta, self.L, self.eps = scenario.POSE_REDCHO_THETA, scenario.POSE_REDCHO_L, scenario.POSE_REDCHO_EPS
        _, self.G = build_redcho_matrices(self.gamma)
        self.X = np.zeros((3, 3))
        self.Z = np.zeros((3, 3))
        self.previous_velocity = np.zeros(2)
        self.cycle, self.finished = 0, True

    def prepare(self, cycle, t, local_velocity_svf):
        if cycle != self.cycle+1 or not self.finished:
            raise ValueError('Position REDCHO cycle order violation')
        if not np.isfinite(t) or abs(t-(cycle-1)*self.dt) > 1e-9:
            raise ValueError('Position REDCHO time mismatch')
        z = np.asarray(local_velocity_svf, float)
        if z.shape != (3, 3) or not np.isfinite(z).all():
            raise ValueError('Position REDCHO needs own finite 3 x 3 SVF state')
        self.cycle, self.finished = cycle, False
        if cycle > 1:
            self.Z[:2, 0] += .5*self.dt*(self.previous_velocity+z[:2, 0])
        self.previous_velocity = z[:2, 0].copy()
        self.Z[:2, 1], self.Z[:2, 2] = z[:2, 0], z[:2, 1]
        hz, vz, az, jz = height_proposal(t, self.agent_id)
        self.Z[2] = [hz, vz, az]
        third = np.r_[z[:2, 2], jz]
        # (D + gamma0)(D + gamma1)(D + gamma2) r_i, m=2.
        c = np.poly(-self.gamma)[1:][::-1]
        self.regularity = third+self.Z@c
        if np.linalg.norm(self.Z[:, 0]) > scenario.POSE_REFERENCE_BOUND:
            raise RuntimeError('Position reference exceeds the configured bounded horizon')
        if np.linalg.norm(self.regularity) > self.L/2.:
            raise RuntimeError('Position REDCHO regularity budget exceeded; do not silently increase L')
        self.Y = self.Z-(self.G@self.X.T).T
        self.result = dict(pose_local_reference_hist=self.Z.copy(),
                           pose_redcho_estimate_hist=self.Y.copy(),
                           pose_regularity_hist=self.regularity.copy())
        return self.Y[:, 0].copy()

    def finish(self, peer_estimates):
        if self.finished or set(peer_estimates) != set(self.neighbors):
            raise ValueError('Position REDCHO requires both same-cycle ring peers')
        adjacency = np.zeros((6, 6), int)
        Y, X = np.zeros((3, 3, 6)), np.zeros((3, 3, 6))
        Y[:, :, self.index], X[:, :, self.index] = self.Y, self.X
        for peer in sorted(peer_estimates):
            value = np.asarray(peer_estimates[peer], float)
            if value.shape != (3,) or not np.isfinite(value).all():
                raise ValueError('Invalid position REDCHO peer estimate')
            adjacency[self.index, peer-1] = 1
            Y[:, 0, peer-1] = value
        self.X += self.dt*redcho_state_derivative(
            X, Y, adjacency, self.gamma, self.theta, self.gains, self.L, self.eps)[:, :, self.index]
        if not np.isfinite(self.X).all():
            raise RuntimeError('Nonfinite position REDCHO state')
        self.finished = True
        return self.result

"""Local battery-reference SVF feeding exactly one position REDCHO bank."""
import numpy as np
from safe_redcho_swarm.core.reference_filters import svf_step
from .pose_redcho import PoseREDCHO


class SinglePositionReference:
    banks_per_agent = 1

    def __init__(self, agent_id):
        self.bank = PoseREDCHO(agent_id)
        self.Z = np.zeros((3,3,1))
        self.snap = np.zeros(3)

    @property
    def cycle(self): return self.bank.cycle

    @property
    def neighbors(self): return self.bank.neighbors

    def prepare(self, cycle, t, raw):
        if not self.bank.finished or cycle != self.bank.cycle+1:
            raise ValueError('Single-position reference cycle order violation')
        raw = np.asarray(raw, float)
        if raw.shape != (3,) or not np.isfinite(raw).all():
            raise ValueError('Require own finite battery velocity reference')
        self.Z, snap = svf_step(self.Z, raw[:,None], 2., self.bank.dt)
        self.snap = snap[:,0].copy()
        return self.bank.prepare(cycle, t, self.Z[:,:,0])

    def finish(self, peers):
        result = self.bank.finish(peers)
        return dict(**result, U_hist=self.Z[:,:,0].copy(), svf_snap_hist=self.snap.copy())

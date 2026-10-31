"""Own SVF/REDCHO states only; REDCHO receives two neighbors' velocity estimates."""
import numpy as np
from safe_redcho_swarm.core.config import Battery
from safe_redcho_swarm.core.battery import speed_governor
from safe_redcho_swarm.core.reference_filters import svf_step,levant_step
from safe_redcho_swarm.core.redcho import build_redcho_matrices,redcho_state_derivative
from .ring_protocol import neighbors


class LocalObserver:
    def __init__(self,agent_id):
        self.agent_id=agent_id;self.index=agent_id-1
        self.neighbors=neighbors(agent_id)
        self.dt=.01;self.L=27.43;self.omega=2.
        self.gamma=np.full(3,.15);self.theta=1.2;self.gains=np.array([2.,1.4,.8]);self.eps=.04
        _,self.G=build_redcho_matrices(self.gamma)
        self.Z=np.zeros((3,3,1));self.Zlev=self.Z.copy()
        soc=np.array([1.,.95,.90,.65,.80,.75])
        self.Z[0,0,0]=0.0  # Resting start; observer states match zero reference
        self.Zlev=self.Z.copy();self.X=np.zeros((3,3));self.Xlev=self.X.copy()
        self.cycle=0;self.finished=True

    def prepare(self,cycle,raw):
        if cycle!=self.cycle+1 or not self.finished:raise ValueError('Observer cycle order violation')
        raw=np.asarray(raw,dtype=float)
        if raw.shape!=(3,) or not np.isfinite(raw).all():raise ValueError('Invalid local raw reference')
        self.cycle=cycle;self.finished=False
        self.Z,snap=svf_step(self.Z,raw[:,None],self.omega,self.dt)
        self.Zlev=levant_step(self.Zlev,raw[:,None],self.L,self.dt)
        self.Y=self.Z[:,:,0]-np.einsum('ab,dbi->dai',self.G,self.X[:,:,None])[:,:,0]
        self.Ylev=self.Zlev[:,:,0]-np.einsum('ab,dbi->dai',self.G,self.Xlev[:,:,None])[:,:,0]
        self.result=dict(svf_state=self.Z[:,:,0].copy(),svf_estimate=self.Y.copy(),
                         levant_state=self.Zlev[:,:,0].copy(),levant_estimate=self.Ylev.copy(),snap=snap[:,0].copy())
        return self.Y[:,0].copy(),self.Ylev[:,0].copy()

    def finish(self,peer_messages):
        if self.finished or set(peer_messages)!=set(self.neighbors):raise ValueError('Expected exactly two neighbors for current cycle')
        # Preserve the baseline function and its ascending agent-ID summation.
        # These scratch arrays contain own state and two received columns only;
        # no other agents' observer states are computed or stored locally.
        adjacency=np.zeros((6,6),dtype=int)
        Ys=np.zeros((3,3,6));Yl=np.zeros_like(Ys)
        Ys[:,:,self.index]=self.Y;Yl[:,:,self.index]=self.Ylev
        for peer,values in peer_messages.items():
            a,b=(np.asarray(x,dtype=float) for x in values)
            if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite(np.r_[a,b]).all():raise ValueError('Invalid neighbor estimates')
            adjacency[self.index,peer-1]=1
            Ys[:,0,peer-1]=a;Yl[:,0,peer-1]=b
        X=np.zeros((3,3,6));Xlev=np.zeros_like(X)
        X[:,:,self.index]=self.X;Xlev[:,:,self.index]=self.Xlev
        self.X+=self.dt*redcho_state_derivative(X,Ys,adjacency,self.gamma,self.theta,self.gains,self.L,self.eps)[:,:,self.index]
        self.Xlev+=self.dt*redcho_state_derivative(Xlev,Yl,adjacency,self.gamma,self.theta,self.gains,self.L,self.eps)[:,:,self.index]
        self.finished=True
        return self.result

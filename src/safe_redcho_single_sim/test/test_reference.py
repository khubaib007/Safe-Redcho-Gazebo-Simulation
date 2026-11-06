import numpy as np
import pytest
from safe_redcho_swarm.core.reference_filters import svf_step
from safe_redcho_single_sim.reference import SinglePositionReference
from safe_redcho_single_sim.local_controller import LocalController
from safe_redcho_single_sim.pose_redcho import PoseREDCHO,regularity_certificate
from safe_redcho_single_sim import scenario


def test_one_position_bank_and_no_velocity_or_levant_bank():
    a=LocalController(1)
    assert isinstance(a.reference.bank,PoseREDCHO) and a.reference.bank.X.shape==(3,3)
    assert not hasattr(a,'observer') and not hasattr(a,'pose_observer')
    assert not hasattr(a.reference,'Xlev') and not hasattr(a.reference,'Zlev')


def test_independent_vectorized_observer_oracle_and_mean_conservation():
    agents={i:SinglePositionReference(i) for i in range(1,7)}
    X=np.zeros((3,3,6));Z=np.zeros_like(X);svf=np.zeros_like(X);previous=np.zeros((2,6))
    G=np.array([[1.,0,0],[-.15,1,0],[.0225,-.3,1]])
    max_difference=0.;tail=[]
    for k in range(4001):
        t=k*.01;raw=np.zeros((3,6))
        if t>=5:raw[0]=np.linspace(.35,.6,6)
        if t>=17:raw[0,1]=.25
        packets={i:a.prepare(k+1,t,raw[:,i-1]) for i,a in agents.items()}
        svf,_=svf_step(svf,raw,2.,.01)
        if k:Z[:2,0]+=.005*(previous+svf[:2,0])
        previous=svf[:2,0].copy();Z[:2,1]=svf[:2,0];Z[:2,2]=svf[:2,1]
        T=scenario.TAKEOFF_END-scenario.TAKEOFF_START;s=np.clip((t-scenario.TAKEOFF_START)/T,0,1)
        dh=scenario.HEIGHT_PROPOSALS-scenario.GROUND_COM_HEIGHT
        Z[2,0]=dh*(10*s**3-15*s**4+6*s**5)
        Z[2,1]=dh*(30*s*s-60*s**3+30*s**4)/T
        Z[2,2]=dh*(60*s-180*s*s+120*s**3)/T**2
        Y=Z-np.einsum('ab,dbi->dai',G,X)
        # Independent analytic ring update, not the production REDCHO kernel.
        dx=-.15*X;dx[:,:2]+=X[:,1:]
        for i in range(6):
            for j in sorted(((i-1)%6,(i+1)%6)):
                e=Y[:,0,i]-Y[:,0,j]
                for mu,gain in enumerate([2.,1.4,.8]):
                    alpha=(2-mu)/3;beta=(mu+1)/3
                    dx[:,mu,i]+=gain*1.2**(mu+1)*32.**beta*e*(e*e+.04**2)**((alpha-1)/2)
        X+=.01*dx
        for i in range(6,0,-1):
            result=agents[i].finish({j:packets[j] for j in agents[i].neighbors})
            max_difference=max(max_difference,float(np.abs(result['pose_redcho_estimate_hist']-Y[:,:,i-1]).max()),float(np.abs(agents[i].bank.X-X[:,:,i-1]).max()))
        assert np.max(np.abs(Y.mean(axis=2)-Z.mean(axis=2)))<1e-11
        if t>=35:tail.append(np.linalg.norm(Y[:,0]-Z[:,0].mean(axis=1)[:,None],axis=0).max())
    assert max_difference<1e-10
    assert max(tail)<.01
    heights=scenario.GROUND_COM_HEIGHT+Y[2,0]
    assert np.max(abs(heights-.8))<.001
    assert abs(heights[0]-scenario.HEIGHT_PROPOSALS[0])>.015


@pytest.mark.parametrize('agent',[0,7,True,1.5])
def test_invalid_agent_id(agent):
    with pytest.raises(ValueError):SinglePositionReference(agent)


@pytest.mark.parametrize('case',['missing','nan','wrong_shape'])
def test_missing_or_invalid_neighbor_rejected(case):
    r=SinglePositionReference(1);r.prepare(1,0.,np.zeros(3))
    peers={2:np.zeros(3),6:np.zeros(3)}
    if case=='missing':peers.pop(6)
    elif case=='nan':peers[6][0]=np.nan
    else:peers[6]=np.zeros(9)
    with pytest.raises(ValueError):r.finish(peers)


def test_regular_reference_budget_and_cycle_guards():
    c=regularity_certificate();assert c['disagreement_bound']<32
    r=SinglePositionReference(1)
    with pytest.raises(ValueError):r.prepare(2,.01,np.zeros(3))
    r.prepare(1,0.,np.zeros(3))
    with pytest.raises(ValueError):r.prepare(2,.01,np.zeros(3))
    r.finish({2:np.zeros(3),6:np.zeros(3)})
    with pytest.raises(ValueError):r.finish({2:np.zeros(3),6:np.zeros(3)})

import csv
import json
import numpy as np
import pytest
from safe_redcho_single_sim.formation_control import apply_formation
from safe_redcho_single_sim.local_controller import LocalController,initial_plant_state
from safe_redcho_single_sim.offline import run
from safe_redcho_single_sim.history import AGENT_SHAPES,allocate_history,store_agent
from safe_redcho_single_sim.protocol import parse_identity,identity,validate_duration
from safe_redcho_single_sim import scenario


def test_all_velocity_and_acceleration_axes_come_from_position_bank():
    pr=np.zeros((3,3));pr[:,1]=[.03,-.04,.02];pr[:,2]=[.08,.07,-.03]
    p=scenario.INITIAL_POSITIONS[:,0];v=np.zeros(3)
    u,d=apply_formation(1,8.,pr,p,v)
    expected=pr[:,2]+4.5*pr[:,1]+.45*np.tanh(pr[:,1])
    np.testing.assert_allclose(u,expected)
    np.testing.assert_array_equal(d['flight_velocity_target_hist'],pr[:,1])
    pr[:,2]+=[.01,.02,.03];u2,_=apply_formation(1,8.,pr,p,v)
    np.testing.assert_allclose(u2-u,[.01,.02,.03])


@pytest.mark.parametrize('case',['nan_state','previous_mismatch','wrong_bound','missing_peer'])
def test_invalid_control_context_fails_closed(case):
    a=LocalController(1);state=initial_plant_state();previous=np.zeros((3,6))
    if case=='nan_state':
        state[0][0,0]=np.nan
        with pytest.raises(ValueError):a.prepare(1,0.,state,previous)
        return
    if case=='previous_mismatch':
        previous[0,0]=1
        with pytest.raises(ValueError):a.prepare(1,0.,state,previous)
        return
    _,b=a.prepare(1,0.,state,previous);peers={2:np.zeros(3),6:np.zeros(3)};bounds=np.full(6,b)
    if case=='wrong_bound':bounds[0]+=.1
    else:peers.pop(6)
    with pytest.raises(ValueError):a.finish(peers,bounds)


def test_short_offline_telemetry_has_real_single_bank_fields(tmp_path):
    s=run(.01,tmp_path,plots=False,csv=True)
    assert all(s['checks'].values()) and not s['recovery_evaluated']
    with np.load(tmp_path/'simulation_data.npz',allow_pickle=False) as h:
        assert h['pose_redcho_estimate_hist'].shape==(3,3,6,2)
        assert 'Y_hist' not in h and 'U_levant_hist' not in h
        assert all(np.isfinite(h[key]).all() for key in h)
    with (tmp_path/'agent_telemetry.csv').open() as f:rows=list(csv.DictReader(f))
    assert len(rows)==12 and {r['agent_id'] for r in rows}==set('123456')
    assert json.loads((tmp_path/'telemetry_schema.json').read_text())['redcho_orders']==['position_m','velocity_mps','acceleration_mps2']


def test_logging_rejects_fabricated_extra_bank_fields():
    h=allocate_history(0.)
    with pytest.raises(ValueError):store_agent(h,0,1,{**{k:np.zeros(s) for k,s in AGENT_SHAPES.items()},'Y_hist':np.zeros((3,3))})


@pytest.mark.parametrize('value',[float('nan'),-.1,.015])
def test_bad_duration(value):
    with pytest.raises(ValueError):validate_duration(value)


@pytest.mark.parametrize('field,value',[('banks_per_agent',2),('architecture','dual'),('implementation','old'),('pid',-1)])
def test_wrong_implementation_identity_rejected(field,value):
    d=identity(123);d[field]=value
    with pytest.raises(ValueError):parse_identity(json.dumps(d))

"""Compare six independent local observers with recorded full-mission outputs."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from safe_redcho_distributed.local_observer import LocalObserver


def run(reference):
    agents={i:LocalObserver(i) for i in range(1,7)}
    mapping={'svf_state':'U_hist','svf_estimate':'Y_hist','levant_state':'U_levant_hist','levant_estimate':'Y_levant_hist','snap':'svf_snap_hist'}
    maxima={k:0. for k in mapping}
    with np.load(reference,allow_pickle=False) as archive:
        h={key:archive[key] for key in ['time','raw_ref_hist',*mapping.values()]}
        for k in range(len(h['time'])):
            messages={i:a.prepare(k+1,h['raw_ref_hist'][:,i-1,k]) for i,a in agents.items()}
            # Reverse finish order demonstrates independent snapshot-based updates.
            for i in range(6,0,-1):
                a=agents[i];result=a.finish({peer:messages[peer] for peer in a.neighbors})
                for key,history in mapping.items():
                    maxima[key]=max(maxima[key],float(np.max(np.abs(result[key]-h[history][...,i-1,k]))))
        print('cycles',len(h['time']),'maximum differences',maxima)
    assert max(maxima.values())<=1e-10
    a=LocalObserver(1)
    for action in [lambda:a.prepare(2,[0.,0.,0.]),lambda:a.prepare(1,[float('nan'),0.,0.])]:
        try:action()
        except ValueError:pass
        else:raise AssertionError('Invalid input accepted')
    a.prepare(1,[.6,0.,0.])
    try:a.finish({2:messages[2]})
    except ValueError:pass
    else:raise AssertionError('Incomplete neighborhood accepted')
    print('PASS: full-mission equivalence and cycle/payload/neighborhood guards')


if __name__=='__main__':
    if len(sys.argv)!=2:raise SystemExit('Usage: python3 check_local_observer.py REFERENCE_NPZ')
    run(sys.argv[1])

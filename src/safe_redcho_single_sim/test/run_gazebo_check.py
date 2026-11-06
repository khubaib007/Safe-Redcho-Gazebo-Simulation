"""Launch only this experiment's native Gazebo/agents in an isolated domain.

No hardware interfaces. Child process group cleanup never touches other runs.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=45.)
    p.add_argument('--plots',action='store_true');args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','42','43','47') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('Use an isolated test domain and ROS_LOCALHOST_ONLY=1')
    if not os.environ.get('GZ_PARTITION','').startswith('single_position_validation_'):
        p.error('Use a unique single_position_validation_ Gazebo partition')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if (out/'mission_status.json').exists():p.error('Use a fresh output directory')
    command=['ros2','launch','safe_redcho_single_sim','simulation.launch.py','headless:=true','rviz:=false',
        'plots:='+str(args.plots).lower(),'csv:=true','shutdown_on_finish:=true',
        'duration:='+str(args.duration),'output:='+str(out)]
    began=time.monotonic()
    with (out/'launch.log').open('w') as log:
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            code=proc.wait(timeout=600)
            if code:raise RuntimeError('Launch exited with '+str(code))
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
    status=json.loads((out/'mission_status.json').read_text())
    cleanup=json.loads((out/'gazebo_shutdown.json').read_text())
    assert not cleanup['process_group_remaining'],cleanup
    assert status['status']=='PASS',status
    cycles=int(round(args.duration/.01))+1
    assert status['completed_cycles']==status['accepted_motor_batches']==cycles
    assert len(set(status['agent_process_ids'].values()))==6
    assert set(status['results_per_agent'].values())=={cycles}
    with np.load(out/'simulation_data.npz',allow_pickle=False) as arrays:
        assert all(np.isfinite(arrays[key]).all() for key in arrays)
        assert arrays['pose_redcho_estimate_hist'].shape==(3,3,6,cycles)
        assert 'Y_hist' not in arrays and 'Y_levant_hist' not in arrays
    with (out/'agent_telemetry.csv').open() as f:rows=sum(1 for _ in csv.DictReader(f))
    assert rows==6*cycles,rows
    figures=list(out.glob('figure_*.png'))
    if args.plots:assert len(figures)==8
    report=dict(status='PASS',scope='Actual Gazebo native physics and six independent single-bank ROS controllers; no hardware',
        ROS_DOMAIN_ID=os.environ['ROS_DOMAIN_ID'],GZ_PARTITION=os.environ['GZ_PARTITION'],
        completed_cycles=cycles,agent_csv_rows=rows,figures=len(figures),
        wall_duration_s=time.monotonic()-began,mission_checks=status['checks'],gazebo_cleanup=cleanup)
    (out/'validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()

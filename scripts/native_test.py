"""Run an isolated headless ROS/Gazebo mission and stop only spawned processes."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('suite', choices=['smoke', 'full'], nargs='?', default='smoke')
args = parser.parse_args()
duration = 2 if args.suite == 'smoke' else 45
variant = json.loads((root/'release.json').read_text())['variant']
if variant=='single-position':
    os.execvp('bash',['bash',str(root/'scripts/test_single_native.sh'),args.suite])

output = root/'results'/('native_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
output.mkdir(parents=True)
env = os.environ.copy()
env['ROS_DOMAIN_ID'] = env.get('REDCHO_TEST_DOMAIN_ID', '162')
env['GZ_PARTITION'] = f'redcho_test_{variant}_{os.getpid()}'
env['ROS_LOG_DIR'] = str(output/'ros_logs')
commands = [
    ('world', ['bash','scripts/run.sh','world','headless:=true','rviz:=false']),
    ('agents', ['bash','scripts/run.sh','agents']),
    ('coordinator', ['bash','scripts/run.sh','gazebo','--duration',str(duration),'--no-plots','--output',str(output/'mission')]),
]
processes=[]
logs=[]
status={'status':'FAIL','duration_s':duration,'variant':variant,'ros_domain_id':env['ROS_DOMAIN_ID'],'gazebo_partition':env['GZ_PARTITION']}
try:
    for name, command in commands:
        log=(output/(name+'.log')).open('w');logs.append(log)
        process=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,process))
        print('START',name,'pid',process.pid,flush=True)
    deadline=time.monotonic()+300
    while processes[-1][1].poll() is None:
        if time.monotonic()>deadline:raise RuntimeError('Native mission exceeded 300-second wall timeout')
        for name,process in processes[:-1]:
            if process.poll() is not None:raise RuntimeError(name+' exited early; inspect '+str(output/(name+'.log')))
        time.sleep(.2)
    result=processes[-1][1].returncode
    report=output/'mission/mission_status.json'
    if result or not report.is_file():raise RuntimeError('Coordinator failed; inspect '+str(output/'coordinator.log'))
    mission=json.loads(report.read_text())
    if mission.get('status')!='PASS':raise RuntimeError('Native mission did not pass')
    status.update(status='PASS',mission=mission)
except BaseException as exc:
    status['error']=type(exc).__name__+': '+str(exc)
finally:
    # Each launch owns its own session. Never terminate unrelated ROS/Gazebo jobs.
    for name,process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
    for name,process in reversed(processes):
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
    for log in logs:log.close()
    (output/'native_test_report.json').write_text(json.dumps(status,indent=2)+'\n')
    print(json.dumps(status,indent=2));print('Native evidence:',output,flush=True)
raise SystemExit(0 if status['status']=='PASS' else 1)

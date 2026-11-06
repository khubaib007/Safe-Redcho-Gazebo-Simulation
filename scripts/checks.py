"""Explicit supported test suite; each protocol check gets an isolated process."""
import argparse,datetime,json,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('suite',choices=['quick','full'],default='quick',nargs='?');args=p.parse_args()
out=root/'results'/('tests_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'));out.mkdir(parents=True)
tests=['src/safe_redcho_distributed/test/check_ring_protocol.py','src/safe_redcho_distributed/test/check_qp_failure_guards.py','src/safe_redcho_distributed/test/check_coordinator_protocol.py','src/safe_redcho_swarm/test/check_lockstep_protocol.py','src/safe_redcho_scene/test/check_scene.py']
if (root/'src/safe_redcho_distributed/test/check_redcho_tracking.py').exists():tests.extend(['src/safe_redcho_distributed/test/check_redcho_tracking.py','src/safe_redcho_distributed/test/check_control_agent_protocol.py'])
commands=[['scripts/verify.py']]+[[t] for t in tests]
if json.loads((root/'release.json').read_text())['variant']=='single-position':commands.append(['-m','pytest','src/safe_redcho_single_sim/test','-q','-p','no:cacheprovider'])
if args.suite=='full':commands.append(['scripts/numerical.py','--duration','45','--output',str(out/'numerical'),'--plots'])
results=[]
for index,command in enumerate(commands):
    print('RUN',*command,flush=True)
    with (out/f'{index:02d}.log').open('w') as log:
        try:
            result=subprocess.run([sys.executable,*command],cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=600)
            code=result.returncode
        except subprocess.TimeoutExpired:code=124
    results.append(dict(command=command,exit_code=code,log=f'{index:02d}.log'))
    print('PASS' if code==0 else 'FAIL',command[0],flush=True)
report=dict(status='PASS' if all(r['exit_code']==0 for r in results) else 'FAIL',suite=args.suite,checks=results)
(out/'test_report.json').write_text(json.dumps(report,indent=2)+'\n');print('Report:',out/'test_report.json');sys.exit(0 if report['status']=='PASS' else 1)

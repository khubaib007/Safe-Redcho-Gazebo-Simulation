"""Own the Gazebo process group, including the Ruby CLI's native child.

Signals target only the fresh session created here. Linux subreaping permits
adopted native descendants to be reaped if the CLI wrapper exits first.
"""
import argparse
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def enable_subreaper():
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(36,1,0,0,0)!=0:
        error=ctypes.get_errno();raise OSError(error,'Cannot establish Gazebo child ownership')


def group_alive(pgid):
    try:os.killpg(pgid,0);return True
    except ProcessLookupError:return False


def reap_group(proc):
    proc.poll()  # Reap the direct CLI child using Popen's bookkeeping.
    while True:
        try:
            pid,_=os.waitpid(-proc.pid,os.WNOHANG)
            if not pid:return
        except ChildProcessError:return


def stop_group(proc):
    signals=[]
    for sig,grace in [(signal.SIGINT,2.),(signal.SIGTERM,1.),(signal.SIGKILL,1.)]:
        reap_group(proc)
        if not group_alive(proc.pid):break
        os.killpg(proc.pid,sig);signals.append(sig.name)
        end=time.monotonic()+grace
        while time.monotonic()<end:
            reap_group(proc)
            if not group_alive(proc.pid):break
            time.sleep(.02)
    reap_group(proc)
    remains=group_alive(proc.pid)
    return dict(child_pid=proc.pid,process_group=proc.pid,signals=signals,
                child_returncode=proc.returncode,process_group_remaining=remains)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',type=Path,required=True)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--cleanup-report',type=Path,required=True)
    args=parser.parse_args()
    if not args.world.is_file():parser.error('World does not exist')
    enable_subreaper();stop=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
    command=['gz','sim','-v','3','--force-version','8']
    if args.headless:command.append('-s')
    command.append(str(args.world))
    proc=subprocess.Popen(command,start_new_session=True)
    code=0
    try:
        while not stop.is_set():
            try:code=proc.wait(timeout=.1);break
            except subprocess.TimeoutExpired:pass
    finally:
        report=stop_group(proc)
        report.update(stop_requested=stop.is_set(),GZ_PARTITION=os.environ.get('GZ_PARTITION'))
        args.cleanup_report.parent.mkdir(parents=True,exist_ok=True)
        args.cleanup_report.write_text(json.dumps(report,indent=2)+'\n')
    if report['process_group_remaining']:raise RuntimeError('Owned Gazebo process group did not stop')
    if code and not stop.is_set():raise SystemExit(code if code>0 else 1)


if __name__=='__main__':main()

"""Owned child-group cleanup, without Gazebo, ROS or unrelated processes."""
import os
import signal
import subprocess
import sys
import time
from safe_redcho_single_sim.gazebo_server import stop_group,group_alive


def test_cleanup_terminates_only_its_own_session(tmp_path):
    ready=tmp_path/'ready'
    script='import signal,time,pathlib;signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);pathlib.Path('+repr(str(ready))+').touch();time.sleep(30)'
    owned=subprocess.Popen([sys.executable,'-c',script],start_new_session=True)
    unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True)
    try:
        end=time.monotonic()+2
        while not ready.exists() and time.monotonic()<end:time.sleep(.01)
        assert ready.exists()
        report=stop_group(owned)
        assert not report['process_group_remaining'] and not group_alive(owned.pid)
        assert report['signals']==['SIGINT','SIGTERM','SIGKILL']
        assert unrelated.poll() is None
    finally:
        for proc in (owned,unrelated):
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGKILL)
            proc.wait(timeout=2)

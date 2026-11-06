"""Read-only dependency, frozen-equation and world consistency checks."""
import hashlib
import importlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from safe_redcho_swarm.core.config import Params
from safe_redcho_scene.geometry_guard import validate_world_geometry
from .formation_control import validate_configuration
from .reference import SinglePositionReference
from .pose_redcho import regularity_certificate
from . import scenario


def check_world(path):
    validate_world_geometry(path,scenario)
    world=ET.parse(path).getroot().find('world');params=Params()
    if abs(float(world.findtext('physics/max_step_size'))-.001)>1e-12:
        raise ValueError('Expected 1 ms physics steps')
    plugin=world.find("plugin[@name='safe_redcho_lockstep::LockstepSystem']")
    if plugin is None or plugin.get('filename')!='libsafe_redcho_lockstep_system.so':
        raise ValueError('Native motor-wrench lockstep plugin missing')
    initial=np.fromstring(plugin.findtext('initial_velocities',''),sep=' ')
    if initial.shape!=(18,) or np.any(initial):raise ValueError('Expected resting bootstrap')
    for i in range(1,7):
        inertial=world.find(f"model[@name='cf{i}']/link/inertial")
        for key,expected in [('mass',params.m),('inertia/ixx',params.Ixx),('inertia/iyy',params.Iyy),('inertia/izz',params.Izz)]:
            if abs(float(inertial.findtext(key))-expected)>1e-12:raise ValueError('Physics/controller mass or inertia mismatch')
    return dict(path=str(path),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),physics_step_s=.001)


def run():
    validate_configuration();share=Path(get_package_share_directory('safe_redcho_single_sim'))
    baseline=json.loads((share/'provenance/original_source_sha256.json').read_text())
    core={}
    for name in ('config','battery','reference_filters','redcho','cbf_qp','quadrotor'):
        module=importlib.import_module('safe_redcho_swarm.core.'+name)
        digest=hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
        expected=baseline['src/safe_redcho_swarm/safe_redcho_swarm/core/'+name+'.py']
        if digest!=expected:raise RuntimeError('Shared core changed since baseline snapshot: '+name)
        core[name]=digest
    for module in ('rclpy','safe_redcho_interfaces.srv','safe_redcho_distributed_interfaces.msg'):
        importlib.import_module(module)
    backend=Path(get_package_prefix('safe_redcho_lockstep'))/'lib/libsafe_redcho_lockstep_system.so'
    if not backend.is_file():raise RuntimeError('Build the native lockstep backend')
    worlds=[check_world(p) for p in sorted((share/'worlds').glob('*.sdf'))]
    for w in worlds:
        if w['sha256']!=baseline['src/safe_redcho_scene/worlds/'+Path(w['path']).name]:
            raise RuntimeError('Baseline physical world changed')
    ref=SinglePositionReference(1)
    return dict(status='PASS_LOCAL_SOFTWARE',architecture=scenario.ARCHITECTURE,banks_per_agent=1,
        redcho_state_shape=list(ref.bank.X.shape),core_sha256=core,worlds=worlds,
        regularity_certificate=regularity_certificate(),
        scope='Read-only source/interface/world checks; no ROS graph, Gazebo execution or physical hardware verification')


def main():print(json.dumps(run(),indent=2))


if __name__=='__main__':main()

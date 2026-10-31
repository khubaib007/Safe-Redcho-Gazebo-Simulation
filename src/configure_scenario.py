"""Regenerate both Gazebo worlds from the common controller scenario settings."""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as E

root=Path(__file__).resolve().parent
path=root/'safe_redcho_distributed/safe_redcho_distributed/reactive_scenario.py'
spec=importlib.util.spec_from_file_location('scenario',path);s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
for file in (root/'safe_redcho_scene/worlds').glob('*.sdf'):
    tree=E.parse(file);w=tree.getroot().find('world')
    for i in range(6):
        model=w.find(f"model[@name='cf{i+1}']")
        model.find('pose').text=' '.join(map(str,[*s.INITIAL_POSITIONS[:,i],0,0,0]))
        model.find('link/collision/geometry/box/size').text='0.09 0.09 0.03485'
    velocities=w.find('.//initial_velocities')
    if velocities is None:raise ValueError('Missing native bootstrap configuration')
    velocities.text=' '.join(['0']*18)
    for i in range(2):
        model=w.find(f"model[@name='obstacle_{i+1}']")
        model.find('pose').text=' '.join(map(str,[*s.OBSTACLE_CENTERS[:,i],0,0,0]))
        for radius in model.findall('.//sphere/radius'):radius.text=str(s.OBSTACLE_RADIUS)
    if file.name=='swarm_closeup.sdf':
        w.find("gui/plugin[@filename='MinimalScene']/camera_pose").text='-2.7 -6.8 1.5 0 0.77 2.12'
    E.indent(tree);tree.write(file,encoding='utf-8',xml_declaration=True)
    print('Updated',file.name)

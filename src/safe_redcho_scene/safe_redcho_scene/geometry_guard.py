"""Reject a stale world before Gazebo starts; no ROS imports are needed."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def validate_world_geometry(path, scenario):
    world = ET.parse(path).getroot().find('world')
    if world is None:
        raise RuntimeError('Gazebo world is missing: '+str(path))
    for i in range(6):
        model = world.find(f"model[@name='cf{i+1}']")
        pose = np.fromstring(model.findtext('pose', '') if model is not None else '', sep=' ')
        expected = np.r_[scenario.INITIAL_POSITIONS[:, i], np.zeros(3)]
        if pose.shape != (6,) or not np.allclose(pose, expected, rtol=0, atol=1e-10):
            raise RuntimeError(f'{Path(path).name}: cf{i+1} initial pose differs from reactive_scenario.py; regenerate worlds and rebuild safe_redcho_scene')
    for o in range(2):
        model = world.find(f"model[@name='obstacle_{o+1}']")
        pose = np.fromstring(model.findtext('pose', '') if model is not None else '', sep=' ')
        radii = [float(r.text) for r in model.findall('.//sphere/radius')] if model is not None else []
        if (pose.shape != (6,) or not np.allclose(pose[:3], scenario.OBSTACLE_CENTERS[:, o], rtol=0, atol=1e-10)
                or not radii or not np.allclose(radii, scenario.OBSTACLE_RADIUS, rtol=0, atol=1e-10)):
            raise RuntimeError(f'{Path(path).name}: obstacle {o+1} geometry differs from reactive_scenario.py; run src/configure_scenario.py and rebuild safe_redcho_scene')

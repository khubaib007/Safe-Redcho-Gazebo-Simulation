from setuptools import find_packages,setup
from glob import glob
setup(name='safe_redcho_distributed',version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/safe_redcho_distributed']),
 ('share/safe_redcho_distributed',['package.xml']),
 ('share/safe_redcho_distributed/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='Khubaib Ahmad',
 maintainer_email='khubaib@todo.todo',description='Distributed ring synchronization',license='Apache-2.0',
 entry_points={'console_scripts':['agent_probe = safe_redcho_distributed.agent_probe:main',
 'ring_check = safe_redcho_distributed.ring_check:main',
 'observer_agent = safe_redcho_distributed.observer_agent:main',
 'observer_check = safe_redcho_distributed.observer_check:main',
 'control_agent = safe_redcho_distributed.control_agent:main',
 'control_replay = safe_redcho_distributed.control_replay:main',
 'gazebo_mission = safe_redcho_distributed.gazebo_mission:main']})

from glob import glob
from setuptools import find_packages, setup

name = 'safe_redcho_single_sim'
setup(name=name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+name]),
                  ('share/'+name, ['package.xml', 'LICENSE', 'README.md', 'ANALYSIS.md', 'MATHEMATICAL_FORMULATION.md', 'RUN_COMMANDS.md', 'VALIDATION.md']),
                  *[('share/'+name+'/'+d, glob(d+'/*')) for d in ('launch','config','worlds','rviz','provenance')]],
      install_requires=['setuptools','numpy','scipy','matplotlib'], zip_safe=True,
      maintainer='Khubaib Ahmad', maintainer_email='maintainer@example.invalid',
      description='Six-agent Gazebo baseline with one position REDCHO bank per aircraft',
      license='Apache-2.0', entry_points={'console_scripts': [
          'control_agent = safe_redcho_single_sim.agent:main',
          'gazebo_mission = safe_redcho_single_sim.gazebo_mission:main',
          'gazebo_server = safe_redcho_single_sim.gazebo_server:main',
          'offline = safe_redcho_single_sim.offline:main',
          'doctor = safe_redcho_single_sim.doctor:main',
          'swarm_view = safe_redcho_single_sim.swarm_view:main']})

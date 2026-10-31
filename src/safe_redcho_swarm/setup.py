from setuptools import find_packages, setup
from glob import glob

package_name = 'safe_redcho_swarm'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khubaib Ahmad',
    maintainer_email='khubaib@todo.todo',
    description='Battery-aware SVF-REDCHO and robust HOCBF swarm control',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'main = safe_redcho_swarm.main:main',
        'baseline = safe_redcho_swarm.nodes.baseline:main',
        'hover_check = safe_redcho_swarm.nodes.hover_check:main',
        'mission = safe_redcho_swarm.nodes.mission:main',
        'lockstep_hover = safe_redcho_swarm.nodes.lockstep_hover:main',
    ]},
)

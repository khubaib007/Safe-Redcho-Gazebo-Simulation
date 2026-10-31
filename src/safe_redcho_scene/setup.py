from setuptools import setup
from glob import glob
setup(name='safe_redcho_scene',version='0.1.0',packages=['safe_redcho_scene'],
 data_files=[('share/ament_index/resource_index/packages',['resource/safe_redcho_scene']),
 ('share/safe_redcho_scene',['package.xml']),
 ('share/safe_redcho_scene/licenses',glob('licenses/*')),
 ('share/safe_redcho_scene/reference',glob('reference/*')),
 ('share/safe_redcho_scene/launch',glob('launch/*.py')),
 ('share/safe_redcho_scene/worlds',glob('worlds/*.sdf')),
 ('share/safe_redcho_scene/meshes',glob('meshes/*.dae')),
 ('share/safe_redcho_scene/rviz',glob('rviz/*.rviz'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='Khubaib Ahmad',
 maintainer_email='khubaib@todo.todo',description='Render-only swarm arena and RViz telemetry',license='Apache-2.0',
 entry_points={'console_scripts':['swarm_view = safe_redcho_scene.swarm_view:main']})

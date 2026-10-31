"""Step 4: motor-thrust plugin, physical state, and diagnostic hover."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('safe_redcho_swarm'))
    gz_launch=Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py'
    plugin_dir=str(Path(get_package_prefix('safe_redcho_gazebo'))/'lib')
    headless=LaunchConfiguration('headless')
    args=PythonExpression(["'-r -v 3 ' + ('-s ' if '",headless,"'.lower() == 'true' else '') + '",str(share/'worlds/single_cf_hover.sdf'),"'"])
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_launch)),launch_arguments={'gz_args':args}.items())
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='safe_redcho_state_bridge',output='screen',
                parameters=[{'config_file':str(share/'config/single_cf_bridge.yaml'),'use_sim_time':True}])
    hover=Node(package='safe_redcho_swarm',executable='hover_check',name='safe_redcho_hover_check',output='screen',
               parameters=[{'use_sim_time':True,'target_height':1.0}])
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='false'),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH',value=[plugin_dir,':',EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',default_value='')]),
        gazebo,bridge,hover])

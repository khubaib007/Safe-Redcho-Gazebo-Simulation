from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression,EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from safe_redcho_scene.geometry_guard import validate_world_geometry
from safe_redcho_distributed import scenario

def generate_launch_description():
    share=Path(get_package_share_directory('safe_redcho_scene'))
    for world_file in (share/'worlds').glob('*.sdf'):
        validate_world_geometry(world_file,scenario)
    original=Path(get_package_share_directory('safe_redcho_swarm'))
    view=LaunchConfiguration('view');headless=LaunchConfiguration('headless')
    args=PythonExpression(["'-v 3 ' + ('-s ' if '",headless,"'.lower() == 'true' else '') + '",str(share/'worlds'),"/' + ('swarm_closeup.sdf' if '",view,"' == 'closeup' else 'swarm_arena.sdf')"])
    return LaunchDescription([
        DeclareLaunchArgument('view',default_value='closeup',choices=['closeup','overview']),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',[str(Path(get_package_prefix('safe_redcho_lockstep'))/'lib'),':',EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',default_value='')]),
        SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH',[str(Path(get_package_prefix('safe_redcho_visualization'))/'lib'),':',EnvironmentVariable('GZ_GUI_PLUGIN_PATH',default_value='')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',[str(share.parent),':',str(original.parent),':',EnvironmentVariable('GZ_SIM_RESOURCE_PATH',default_value='')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':args}.items()),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='safe_redcho_scene_clock',output='screen',parameters=[{'config_file':str(original/'config/lockstep_bridge.yaml'),'use_sim_time':True}]),
        Node(package='safe_redcho_scene',executable='swarm_view',output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/swarm.rviz')],condition=IfCondition(LaunchConfiguration('rviz')),parameters=[{'use_sim_time':True}])])

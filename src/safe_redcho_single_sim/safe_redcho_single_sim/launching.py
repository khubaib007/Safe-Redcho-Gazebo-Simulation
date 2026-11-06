"""New package launches; original worlds/controllers are never edited."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,SetEnvironmentVariable,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.substitutions import LaunchConfiguration,EnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from .doctor import check_world


def description(*, agents=True, mission=True):
    def setup(context):
        value=lambda key:LaunchConfiguration(key).perform(context)
        enabled=lambda key:value(key).lower()=='true'
        share=Path(get_package_share_directory('safe_redcho_single_sim'))
        world=share/'worlds'/('swarm_closeup.sdf' if value('view')=='closeup' else 'swarm_arena.sdf')
        check_world(world)
        scene=Path(get_package_share_directory('safe_redcho_scene'))
        original=Path(get_package_share_directory('safe_redcho_swarm'))
        gzcmd=[str(Path(get_package_prefix('safe_redcho_single_sim'))/'lib/safe_redcho_single_sim/gazebo_server'),
               '--world',str(world),'--cleanup-report',str(Path(value('output'))/'gazebo_shutdown.json')]
        if enabled('headless'):gzcmd.append('--headless')
        actions=[
            SetEnvironmentVariable('ROS_DOMAIN_ID',value('ros_domain')),
            SetEnvironmentVariable('GZ_PARTITION',value('partition')),
            SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',[str(Path(get_package_prefix('safe_redcho_lockstep'))/'lib'),':',EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',default_value='')]),
            SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH',[str(Path(get_package_prefix('safe_redcho_visualization'))/'lib'),':',EnvironmentVariable('GZ_GUI_PLUGIN_PATH',default_value='')]),
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',[str(share.parent),':',str(scene.parent),':',str(original.parent),':',EnvironmentVariable('GZ_SIM_RESOURCE_PATH',default_value='')]),
            ExecuteProcess(cmd=gzcmd,name='single_position_gazebo',output='screen',sigterm_timeout='8',sigkill_timeout='2'),
            Node(package='ros_gz_bridge',executable='parameter_bridge',name='single_position_clock',output='screen',parameters=[{'config_file':str(share/'config/clock_bridge.yaml')}]),
            Node(package='safe_redcho_single_sim',executable='swarm_view',output='screen')]
        if enabled('rviz'):
            actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/swarm.rviz')],parameters=[{'use_sim_time':True}]))
        if agents:
            actions.extend(Node(package='safe_redcho_single_sim',executable='control_agent',namespace=f'single_cf{i}',
                output='screen',parameters=[{'agent_id':i}]) for i in range(1,7))
        if mission:
            args=['--duration',value('duration'),'--output',value('output')]
            if not enabled('plots'):args.append('--no-plots')
            if not enabled('csv'):args.append('--no-csv')
            runner=Node(package='safe_redcho_single_sim',executable='gazebo_mission',output='screen',arguments=args)
            if enabled('shutdown_on_finish'):
                actions.append(RegisterEventHandler(OnProcessExit(target_action=runner,
                    on_exit=[EmitEvent(event=Shutdown(reason='Single-position mission process finished; inspect mission_status.json'))])))
            actions.append(runner)
        return actions
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('view',default_value='closeup',choices=['closeup','overview']),
        DeclareLaunchArgument('plots',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('csv',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('duration',default_value='45.0'),
        DeclareLaunchArgument('output',default_value=str(Path.home()/'safe_redcho_distributed_ws/results/single_position_gazebo')),
        DeclareLaunchArgument('ros_domain',default_value=EnvironmentVariable('ROS_DOMAIN_ID',default_value='47')),
        DeclareLaunchArgument('partition',default_value=EnvironmentVariable('GZ_PARTITION',default_value='safe_redcho_single_sim')),
        DeclareLaunchArgument('shutdown_on_finish',default_value='false',choices=['true','false']),
        OpaqueFunction(function=setup)])

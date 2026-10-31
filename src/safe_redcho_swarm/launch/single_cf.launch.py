"""Step 3: one unpowered Crazyflie body, physics, clock, and ROS odometry."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('safe_redcho_swarm'))
    gazebo_launch = Path(get_package_share_directory('ros_gz_sim'))/'launch'/'gz_sim.launch.py'
    headless = LaunchConfiguration('headless')
    world = str(share/'worlds'/'single_cf.sdf')
    # -r starts physics; -s is server-only when headless:=true.
    gz_args = PythonExpression(["'-r -v 3 ' + ('-s ' if '", headless, "'.lower() == 'true' else '') + '", world, "'"])
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_launch)),
                                     launch_arguments={'gz_args': gz_args}.items())
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='safe_redcho_state_bridge',
                  output='screen', parameters=[{'config_file': str(share/'config'/'single_cf_bridge.yaml'),
                                               'use_sim_time': True}])
    return LaunchDescription([DeclareLaunchArgument('headless', default_value='false',
                                                    description='Run Gazebo without GUI'), gazebo, bridge])

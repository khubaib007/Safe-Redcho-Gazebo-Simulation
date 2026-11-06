from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='safe_redcho_single_sim',executable='control_agent',
        namespace=f'single_cf{i}',output='screen',parameters=[{'agent_id':i}]) for i in range(1,7)])

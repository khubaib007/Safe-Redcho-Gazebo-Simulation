"""Six OS processes, each with precisely two ring-neighbor subscriptions."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='safe_redcho_distributed',executable='agent_probe',
        namespace=f'cf{i}',name='safe_redcho_agent_probe',output='screen',
        parameters=[{'agent_id':i}]) for i in range(1,7)])

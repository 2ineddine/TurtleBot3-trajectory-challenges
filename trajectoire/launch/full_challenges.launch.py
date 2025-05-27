from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectoire',
            executable='challenge1',
            name='challenge1_node',
            output='screen'
        ),
        Node(
            package='trajectoire',
            executable='challenge2',
            name='challenge2_node',
            output='screen'
        ),
        Node(
            package='trajectoire',
            executable='challenge3',
            name='challenge3_node',
            output='screen'
        ),
    ])

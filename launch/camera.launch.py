from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='left_camera',
            output='screen',
            parameters=[
                {'camera': 0},
                {'role': 'raw'},
                {'width': 1640},
                {'height': 1232},
                {'format': str('BGR888')},
            ]
        ),
        
        Node(
            package='camera_ros',
            executable='camera_node',
            name='right_camera',
            output='screen',
            parameters=[
                {'camera': 1},
                {'role': 'raw'},
                {'width': 1640},
                {'height': 1232},
            ]
        )
    ])

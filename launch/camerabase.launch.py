from ament_index_python.resources import has_resource

from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description with for the camera node and a visualiser.

    Returns
    -------
        LaunchDescription: the launch description

    """
    # left parameters
    left_camera_param_name = "left_camera"
    left_camera_param_default = str(0)
    left_camera_param = LaunchConfiguration(
        left_camera_param_name,
        default=left_camera_param_default,
    )
    left_camera_launch_arg = DeclareLaunchArgument(
        left_camera_param_name,
        default_value=left_camera_param_default,
        description="left camera ID or name"
    )
    
    format_param_name = "format"
    format_param_default = str('BGR888')
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name,
        default_value=format_param_default,
        description="pixel format"
    )

    # camera node
    left_composable_nodes = [
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='left_camera_node',
            parameters=[{
                "camera": left_camera_param,
                "width": 640,
                "height": 480,
                "format": format_param,
                "role" : 'raw',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        
    ]
    
    # right parameters
    
    right_camera_param_name = "right_camera"
    right_camera_param_default = str(1)
    right_camera_param = LaunchConfiguration(
        right_camera_param_name,
        default=right_camera_param_default,
    )
    right_camera_launch_arg = DeclareLaunchArgument(
        right_camera_param_name,
        default_value=right_camera_param_default,
        description="right camera ID or name"
    )

    # camera node
    right_composable_nodes = [
        ComposableNode(
            package='camera_ros',
            plugin='camera::CameraNode',
            name='right_camera_node',
            parameters=[{
                "camera": right_camera_param,
                "width": 640,
                "height": 480,
                "format": format_param,
                "role" : 'raw',
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        
    ]

    # # optionally add ImageViewNode to show camera image
    # if has_resource("packages", "image_view"):
    #     composable_nodes += [
    #         ComposableNode(
    #             package='image_view',
    #             plugin='image_view::ImageViewNode',
    #             remappings=[('/image', '/camera/image_raw')],
    #             extra_arguments=[{'use_intra_process_comms': True}],
    #         ),
    #     ]

    # composable nodes in single container
    left_container = ComposableNodeContainer(
        name='left_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=left_composable_nodes,
    )
    
    right_container = ComposableNodeContainer(
        name='right_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=right_composable_nodes,
    )

    return LaunchDescription([
        left_container,
        left_camera_launch_arg,
        right_container,
        right_camera_launch_arg,
        format_launch_arg,
    ])
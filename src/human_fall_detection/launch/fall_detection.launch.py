from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('human_fall_detection')
    
    # Default parameters file
    default_params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameters file'
    )
    
    # Nodes to launch
    camera_node = Node(
        package='human_fall_detection',
        executable='camera_node',
        name='camera_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    
    pose_detection_node = Node(
        package='human_fall_detection',
        executable='pose_detection_node',
        name='pose_detection_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    object_detection_node = Node(
        package='human_fall_detection',
        executable='object_detection_node',
        name='object_detection_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    
    fall_detection_node = Node(
        package='human_fall_detection',
        executable='fall_detection_node',
        name='fall_detection_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    visualization_node = Node(
        package='human_fall_detection',
        executable='visualization_node',
        name='visualization_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    
    alarm_node = Node(
        package='human_fall_detection',
        executable='alarm_node',
        name='alarm_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        params_file_arg,
        camera_node,
        pose_detection_node,
        object_detection_node,
        fall_detection_node,
        visualization_node,
        alarm_node
    ])
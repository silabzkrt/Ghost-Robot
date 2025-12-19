from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for Ghost Publisher with parameter file.
    
    This launch file starts the ghost_publisher node using parameters
    from a YAML configuration file.
    
    Example usage:
        ros2 launch ghost_follower ghost_publisher_with_params.launch.py
    """
    
    # Get the package directory
    pkg_dir = get_package_share_directory('ghost_follower')
    params_file = os.path.join(pkg_dir, 'config', 'ghost_publisher_params.yaml')
    
    # Validate that the parameter file exists
    if not os.path.isfile(params_file):
        raise FileNotFoundError(
            f"Parameter file not found: {params_file}\n"
            f"Please ensure the ghost_follower package is properly installed."
        )
    
    # Declare launch argument for custom params file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to the parameters file'
    )
    
    # Define the ghost publisher node
    ghost_publisher_node = Node(
        package='ghost_follower',
        executable='ghost_publisher',
        name='ghost_publisher',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    return LaunchDescription([
        params_file_arg,
        ghost_publisher_node,
    ])

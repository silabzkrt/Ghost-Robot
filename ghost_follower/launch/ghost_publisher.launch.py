from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for the Ghost Publisher node.
    
    This launch file starts the ghost_publisher node which publishes
    geometry_msgs/Twist messages in a figure-8 pattern on the cmd_vel topic.
    
    Launch arguments:
        linear_scale (float): Scale factor for linear velocity (default: 0.5)
        angular_scale (float): Scale factor for angular velocity (default: 1.0)
        frequency (float): Frequency of the figure-8 pattern in Hz (default: 0.5)
        publish_rate_hz (int): Publishing rate in Hz (default: 50)
    
    Example usage:
        ros2 launch ghost_follower ghost_publisher.launch.py
        ros2 launch ghost_follower ghost_publisher.launch.py linear_scale:=0.8 frequency:=0.3
    """
    
    # Declare launch arguments
    linear_scale_arg = DeclareLaunchArgument(
        'linear_scale',
        default_value='0.5',
        description='Scale factor for linear velocity'
    )
    
    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale',
        default_value='1.0',
        description='Scale factor for angular velocity'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='0.5',
        description='Frequency of the figure-8 pattern in Hz'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='50',
        description='Publishing rate in Hz'
    )
    
    # Define the ghost publisher node
    ghost_publisher_node = Node(
        package='ghost_follower',
        executable='ghost_publisher',
        name='ghost_publisher',
        output='screen',
        parameters=[{
            'linear_scale': LaunchConfiguration('linear_scale'),
            'angular_scale': LaunchConfiguration('angular_scale'),
            'frequency': LaunchConfiguration('frequency'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }]
    )
    
    return LaunchDescription([
        linear_scale_arg,
        angular_scale_arg,
        frequency_arg,
        publish_rate_arg,
        ghost_publisher_node,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('twist_to_carla_control')
    
    # Launch arguments
    max_steering_arg = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.7',
        description='Maximum steering angle in radians'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='10.0',
        description='Maximum speed in m/s'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='2.87',
        description='Vehicle wheelbase in meters'
    )

    # Node
    twist_to_carla_node = Node(
        package='twist_to_carla_control',
        executable='twist_to_carla_control_node',
        name='twist_to_carla_control',
        output='screen',
        parameters=[{
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'max_throttle': 1.0,
            'max_brake': 1.0,
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_speed': LaunchConfiguration('max_speed'),
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/carla/hero/vehicle_control_cmd', '/carla/hero/vehicle_control_cmd'),
        ]
    )

    return LaunchDescription([
        max_steering_arg,
        max_speed_arg,
        wheelbase_arg,
        twist_to_carla_node,
    ])

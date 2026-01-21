from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    carla_host_arg = DeclareLaunchArgument(
        'carla_host',
        default_value='localhost',
        description='CARLA server host'
    )
    
    carla_port_arg = DeclareLaunchArgument(
        'carla_port',
        default_value='2000',
        description='CARLA server port'
    )
    
    role_name_arg = DeclareLaunchArgument(
        'role_name',
        default_value='ego_vehicle',
        description='Vehicle role name'
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
    twist_carla_node = Node(
        package='twist_carla_direct_control',
        executable='twist_carla_controller',
        name='twist_carla_controller',
        output='screen',
        parameters=[{
            'carla_host': LaunchConfiguration('carla_host'),
            'carla_port': LaunchConfiguration('carla_port'),
            'role_name': LaunchConfiguration('role_name'),
            'timeout': 10.0,
            'max_steering_angle': 0.7,
            'max_speed': LaunchConfiguration('max_speed'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'update_rate': 50.0,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        carla_host_arg,
        carla_port_arg,
        role_name_arg,
        max_speed_arg,
        wheelbase_arg,
        twist_carla_node,
    ])
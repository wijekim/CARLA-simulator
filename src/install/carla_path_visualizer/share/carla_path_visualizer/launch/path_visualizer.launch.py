#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'carla_host',
            default_value='localhost',
            description='CARLA server host address'
        ),
        DeclareLaunchArgument(
            'carla_port',
            default_value='2000',
            description='CARLA server port'
        ),
        DeclareLaunchArgument(
            'path_topic',
            default_value='/plan',
            description='Path topic name'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry or Pose topic name'
        ),
        DeclareLaunchArgument(
            'use_odometry',
            default_value='true',
            description='Use Odometry (true) or PoseStamped (false)'
        ),
        DeclareLaunchArgument(
            'robot_size',
            default_value='2.0',
            description='Robot marker size in meters'
        ),
        DeclareLaunchArgument(
            'show_trajectory',
            default_value='true',
            description='Show robot trajectory'
        ),
        DeclareLaunchArgument(
            'trajectory_max_points',
            default_value='1000',
            description='Maximum trajectory points to keep'
        ),
        
        # Path Visualizer Node
        Node(
            package='carla_path_visualizer',
            executable='path_visualizer',
            name='carla_path_visualizer',
            output='screen',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'path_topic': LaunchConfiguration('path_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'use_odometry': LaunchConfiguration('use_odometry'),
                'robot_size': LaunchConfiguration('robot_size'),
                'show_trajectory': LaunchConfiguration('show_trajectory'),
                'trajectory_max_points': LaunchConfiguration('trajectory_max_points'),
            }]
        ),
    ])

#!/usr/bin/env python3
"""
Launch file for CARLA Path Planner
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for CARLA path planner
    """
    
    # Declare launch arguments
    carla_host_arg = DeclareLaunchArgument(
        'carla_host',
        default_value='localhost',
        description='CARLA server host address'
    )
    
    carla_port_arg = DeclareLaunchArgument(
        'carla_port',
        default_value='2000',
        description='CARLA server port'
    )
    
    start_spawn_id_arg = DeclareLaunchArgument(
        'start_spawn_id',
        default_value='0',
        description='Starting spawn point ID'
    )
    
    goal_spawn_id_arg = DeclareLaunchArgument(
        'goal_spawn_id',
        default_value='50',
        description='Goal spawn point ID'
    )
    
    # waypoint_ids는 문자열로 전달 (빈 문자열이 기본값)
    waypoint_ids_arg = DeclareLaunchArgument(
        'waypoint_ids',
        default_value='',
        description='Comma-separated list of intermediate waypoint spawn IDs (e.g., "10,20,30")'
    )
    
    path_resolution_arg = DeclareLaunchArgument(
        'path_resolution',
        default_value='1.0',
        description='Distance between path waypoints in meters'
    )
    
    # Create path planner node
    path_planner_node = Node(
        package='carla_path_planner',
        executable='path_planner',
        name='carla_path_planner',
        output='screen',
        parameters=[{
            'carla_host': LaunchConfiguration('carla_host'),
            'carla_port': LaunchConfiguration('carla_port'),
            'start_spawn_id': LaunchConfiguration('start_spawn_id'),
            'goal_spawn_id': LaunchConfiguration('goal_spawn_id'),
            'waypoint_ids': LaunchConfiguration('waypoint_ids'),
            'path_resolution': LaunchConfiguration('path_resolution'),
        }]
    )
    
    return LaunchDescription([
        carla_host_arg,
        carla_port_arg,
        start_spawn_id_arg,
        goal_spawn_id_arg,
        waypoint_ids_arg,
        path_resolution_arg,
        path_planner_node,
    ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Dynamixel Workbench 노드 (C++)
    # 'xterm'을 사용하여 별도의 터미널 창에서 실행되도록 설정합니다.
    dynamixel_node = Node(
        package='dynamixel_workbench_operators', 
        executable='dynamixel_workbench_operators',  # 실제 실행 파일 이름으로 확인 필요
        name='dynamixel_manager',
        output='screen',
        prefix='xterm -hold -e',  # 별도 창 생성 및 종료 후 창 유지
        parameters=[{
            # 'usb_port': '/dev/ttyUSB0',
            # 'baud_rate': 57600,
        }]
    )

    # 2. Twist Carla Direct Control 노드 (Python)
    carla_control_node = Node(
        package='twist_carla_direct_control',
        executable='twist_carla_controller',        # 실제 실행 파일 이름으로 확인 필요
        name='carla_twist_controller',
        output='screen'
    )

    return LaunchDescription([
        dynamixel_node,
        carla_control_node
    ])
#!/usr/bin/env python3
import os
import sys

# 1. Wayland 관련 Qt 경고 메시지 차단
os.environ['QT_LOGGING_RULES'] = 'qt.qpa.wayland=false'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import carla
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
import math

class CarlaPathVisualizer(Node):
    def __init__(self):
        super().__init__('carla_path_visualizer')
        
        # 파라미터 선언
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_odometry', True)  # True: Odometry, False: PoseStamped
        self.declare_parameter('robot_size', 2.0)  # 로봇 표시 크기
        self.declare_parameter('show_trajectory', True)  # 궤적 표시 여부
        self.declare_parameter('trajectory_max_points', 1000)  # 최대 궤적 포인트 수
        
        # 파라미터 가져오기
        carla_host = self.get_parameter('carla_host').value
        carla_port = self.get_parameter('carla_port').value
        path_topic = self.get_parameter('path_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.use_odometry = self.get_parameter('use_odometry').value
        self.robot_size = self.get_parameter('robot_size').value
        self.show_trajectory = self.get_parameter('show_trajectory').value
        self.trajectory_max_points = self.get_parameter('trajectory_max_points').value
        
        # 2. CARLA 연결
        try:
            self.client = carla.Client(carla_host, carla_port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.carla_map = self.world.get_map()
            self.get_logger().info(f"CARLA 서버 연결 성공: {carla_host}:{carla_port}")
        except Exception as e:
            self.get_logger().error(f"CARLA 서버 연결 실패: {e}")
            raise
        
        # 3. Matplotlib 2D 설정
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(14, 14))
        
        # 실시간 업데이트용 객체 (2D)
        self.path_line, = self.ax.plot([], [], 'g-', lw=2, label='Planned Path', zorder=10)
        self.start_point, = self.ax.plot([], [], 'bo', markersize=10, label='Start', zorder=11)
        self.end_point, = self.ax.plot([], [], 'ro', markersize=10, label='Goal', zorder=11)
        
        # 로봇 위치 표시 (파란색 삼각형 + 방향 화살표)
        self.robot_marker, = self.ax.plot([], [], 'b^', markersize=15, label='Robot', zorder=15, markeredgecolor='darkblue', markeredgewidth=2)
        self.robot_arrow = None
        
        # 로봇 궤적 (연한 파란색 선)
        self.robot_trajectory_x = []
        self.robot_trajectory_y = []
        if self.show_trajectory:
            self.trajectory_line, = self.ax.plot([], [], 'c-', lw=1.5, alpha=0.6, label='Trajectory', zorder=8)
        
        # 배경 지도 및 원점 축 그리기
        self.draw_static_map()
        self.draw_origin_axes()
        
        self.ax.set_aspect('equal')
        self.ax.set_title("CARLA ROS2 2D Path Visualizer with Robot Position", fontsize=16, fontweight='bold')
        self.ax.set_xlabel("X (meters)", fontsize=12)
        self.ax.set_ylabel("Y (meters)", fontsize=12)
        self.ax.legend(loc='upper right', fontsize=10)
        
        # 4. 데이터 저장 변수
        self.path_x = []
        self.path_y = []
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        
        # 5. ROS2 구독 및 타이머
        self.subscription_path = self.create_subscription(
            Path, 
            path_topic, 
            self.path_callback, 
            10
        )
        
        # 로봇 위치 구독 (Odometry 또는 PoseStamped)
        if self.use_odometry:
            self.subscription_robot = self.create_subscription(
                Odometry, 
                odom_topic, 
                self.odom_callback, 
                10
            )
            self.get_logger().info(f"Odometry 토픽 구독: {odom_topic}")
        else:
            self.subscription_robot = self.create_subscription(
                PoseStamped, 
                odom_topic, 
                self.pose_callback, 
                10
            )
            self.get_logger().info(f"PoseStamped 토픽 구독: {odom_topic}")
        
        self.timer = self.create_timer(0.05, self.update_display)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("로봇 위치가 포함된 2D 시각화 노드가 시작되었습니다.")
        self.get_logger().info(f"Path 토픽: {path_topic}")
        self.get_logger().info(f"궤적 표시: {self.show_trajectory}")
        self.get_logger().info("=" * 60)

    def draw_origin_axes(self):
        """원점에 X(빨강), Y(초록) 축 화살표 그리기"""
        length = 20.0
        
        # X축 화살표 (빨강)
        self.ax.quiver(0, 0, length, 0, color='red', angles='xy', scale_units='xy', 
                      scale=1, width=0.006, zorder=5, alpha=0.7)
        # Y축 화살표 (초록)
        self.ax.quiver(0, 0, 0, length, color='green', angles='xy', scale_units='xy', 
                      scale=1, width=0.006, zorder=5, alpha=0.7)
        
        # 축 레이블 텍스트
        self.ax.text(length + 2, 0, "X", color='red', fontweight='bold', 
                    fontsize=14, va='center')
        self.ax.text(0, length + 2, "Y", color='green', fontweight='bold', 
                    fontsize=14, ha='center')

    def draw_static_map(self):
        """도로 차선 및 스폰 포인트 시각화"""
        try:
            # 1. 웨이포인트(도로) 추출
            waypoints = self.carla_map.generate_waypoints(2.0)
            wp_x = [wp.transform.location.x for wp in waypoints]
            wp_y = [-wp.transform.location.y for wp in waypoints]  # CARLA -> ROS2 Y 좌표 반영
            
            # 도로 (연한 회색 점)
            self.ax.scatter(wp_x, wp_y, s=1, c='lightgray', alpha=0.4, zorder=1)
            self.get_logger().info(f"도로 웨이포인트 {len(waypoints)}개 로드 완료")

            # 2. 스폰 포인트 및 숫자
            spawn_points = self.carla_map.get_spawn_points()
            for i, sp in enumerate(spawn_points):
                x, y = sp.location.x, -sp.location.y
                self.ax.plot(x, y, 'rx', markersize=6, alpha=0.6, zorder=2)
                self.ax.text(x + 1, y + 1, str(i), fontsize=9, color='darkred', 
                           fontweight='bold', zorder=3)
            
            self.get_logger().info(f"스폰 포인트 {len(spawn_points)}개 표시 완료")

            # 그리드 설정
            self.ax.grid(True, linestyle=':', alpha=0.4, linewidth=0.5)
            
        except Exception as e:
            self.get_logger().error(f"지도 그리기 실패: {e}")

    def path_callback(self, msg):
        """/plan 토픽 수신"""
        if len(msg.poses) > 0:
            self.path_x = [pose.pose.position.x for pose in msg.poses]
            self.path_y = [pose.pose.position.y for pose in msg.poses]
            self.get_logger().info(f"경로 수신: {len(msg.poses)} 포인트", throttle_duration_sec=2.0)

    def odom_callback(self, msg):
        """/odom 토픽 수신 (Odometry)"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Quaternion을 Yaw로 변환
        orientation = msg.pose.pose.orientation
        self.robot_yaw = self.quaternion_to_yaw(orientation.x, orientation.y, 
                                                 orientation.z, orientation.w)
        
        # 궤적 저장
        if self.show_trajectory:
            self.robot_trajectory_x.append(self.robot_x)
            self.robot_trajectory_y.append(self.robot_y)
            
            # 최대 포인트 수 제한
            if len(self.robot_trajectory_x) > self.trajectory_max_points:
                self.robot_trajectory_x.pop(0)
                self.robot_trajectory_y.pop(0)

    def pose_callback(self, msg):
        """/current_pose 토픽 수신 (PoseStamped)"""
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        
        # Quaternion을 Yaw로 변환
        orientation = msg.pose.orientation
        self.robot_yaw = self.quaternion_to_yaw(orientation.x, orientation.y, 
                                                 orientation.z, orientation.w)
        
        # 궤적 저장
        if self.show_trajectory:
            self.robot_trajectory_x.append(self.robot_x)
            self.robot_trajectory_y.append(self.robot_y)
            
            # 최대 포인트 수 제한
            if len(self.robot_trajectory_x) > self.trajectory_max_points:
                self.robot_trajectory_x.pop(0)
                self.robot_trajectory_y.pop(0)

    def quaternion_to_yaw(self, x, y, z, w):
        """Quaternion을 Yaw 각도로 변환 (라디안)"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def update_display(self):
        """화면 데이터 갱신"""
        try:
            # 경로 업데이트
            if self.path_x:
                self.path_line.set_data(self.path_x, self.path_y)
                self.start_point.set_data([self.path_x[0]], [self.path_y[0]])
                self.end_point.set_data([self.path_x[-1]], [self.path_y[-1]])
            
            # 로봇 위치 업데이트
            if self.robot_x is not None and self.robot_y is not None:
                # 로봇 마커 업데이트
                self.robot_marker.set_data([self.robot_x], [self.robot_y])
                
                # 기존 화살표 제거
                if self.robot_arrow is not None:
                    self.robot_arrow.remove()
                
                # 로봇 방향 화살표 그리기
                if self.robot_yaw is not None:
                    arrow_length = self.robot_size * 3
                    dx = arrow_length * math.cos(self.robot_yaw)
                    dy = arrow_length * math.sin(self.robot_yaw)
                    
                    self.robot_arrow = self.ax.arrow(
                        self.robot_x, self.robot_y, dx, dy,
                        head_width=self.robot_size * 1.5, 
                        head_length=self.robot_size * 1.2,
                        fc='darkblue', ec='navy', linewidth=2, 
                        alpha=0.8, zorder=14
                    )
            
            # 로봇 궤적 업데이트
            if self.show_trajectory and len(self.robot_trajectory_x) > 1:
                self.trajectory_line.set_data(self.robot_trajectory_x, self.robot_trajectory_y)
            
            # 화면 갱신
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f"디스플레이 업데이트 오류: {e}", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CarlaPathVisualizer()
        
        # ROS2 spin과 matplotlib 이벤트 루프를 함께 실행
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.001)
            
    except KeyboardInterrupt:
        print("\n프로그램 종료 중...")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        plt.close('all')
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

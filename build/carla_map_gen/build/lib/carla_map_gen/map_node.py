import os
import sys

# 1. Wayland 관련 Qt 경고 메시지 차단
os.environ['QT_LOGGING_RULES'] = 'qt.qpa.wayland=false'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import carla
import numpy as np
import matplotlib.pyplot as plt

class CarlaPathVisualizer(Node):
    def __init__(self):
        super().__init__('carla_path_visualizer')
        
        # 2. CARLA 연결
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.carla_map = self.world.get_map()
        
        # 3. Matplotlib 2D 설정
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        
        # 실시간 업데이트용 객체 (2D)
        self.path_line, = self.ax.plot([], [], 'g-', lw=2, label='Planned Path', zorder=10)
        self.start_point, = self.ax.plot([], [], 'bo', markersize=8, label='Start')
        self.end_point, = self.ax.plot([], [], 'ro', markersize=8, label='Goal')
        
        # 배경 지도 및 원점 축 그리기
        self.draw_static_map()
        self.draw_origin_axes()
        
        self.ax.set_aspect('equal')
        self.ax.set_title("CARLA ROS2 2D Path Visualizer")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.legend(loc='upper right')
        
        # 4. 데이터 저장 변수
        self.path_x = []
        self.path_y = []

        # 5. ROS2 구독 및 타이머
        self.subscription = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.timer = self.create_timer(0.05, self.update_display)
        
        self.get_logger().info("Z축이 제거된 2D 시각화 노드가 시작되었습니다.")

    def draw_origin_axes(self):
        """원점에 X(빨강), Y(초록) 축 화살표 그리기"""
        # 화살표 길이 (meters)
        length = 20.0
        
        # X축 화살표 (빨강)
        self.ax.quiver(0, 0, length, 0, color='r', angles='xy', scale_units='xy', scale=1, width=0.005, label='X-axis')
        # Y축 화살표 (초록)
        self.ax.quiver(0, 0, 0, length, color='g', angles='xy', scale_units='xy', scale=1, width=0.005, label='Y-axis')
        
        # 축 레이블 텍스트
        self.ax.text(length + 1, 0, "X", color='red', fontweight='bold', va='center')
        self.ax.text(0, length + 1, "Y", color='green', fontweight='bold', ha='center')

    def draw_static_map(self):
        """도로 차선 및 스폰 포인트 시각화"""
        # 1. 웨이포인트(도로) 추출
        waypoints = self.carla_map.generate_waypoints(2.0)
        wp_x = [wp.transform.location.x for wp in waypoints]
        wp_y = [-wp.transform.location.y for wp in waypoints] # CARLA -> ROS2 Y 좌표 반영
        
        # 도로 (연한 회색 점)
        self.ax.scatter(wp_x, wp_y, s=1, c='lightgray', alpha=0.3, zorder=1)

        # 2. 스폰 포인트 및 숫자
        spawn_points = self.carla_map.get_spawn_points()
        for i, sp in enumerate(spawn_points):
            x, y = sp.location.x, -sp.location.y
            self.ax.plot(x, y, 'rx', markersize=5, alpha=0.5, zorder=2)
            self.ax.text(x + 0.5, y + 0.5, str(i), fontsize=8, color='blue', zorder=3)

        # 그리드 설정
        self.ax.grid(True, linestyle=':', alpha=0.5)

    def path_callback(self, msg):
        """/plan 토픽 수신"""
        self.path_x = [pose.pose.position.x for pose in msg.poses]
        self.path_y = [pose.pose.position.y for pose in msg.poses]

    def update_display(self):
        """화면 데이터 갱신"""
        if self.path_x:
            self.path_line.set_data(self.path_x, self.path_y)
            self.start_point.set_data([self.path_x[0]], [self.path_y[0]])
            self.end_point.set_data([self.path_x[-1]], [self.path_y[-1]])
            
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = CarlaPathVisualizer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
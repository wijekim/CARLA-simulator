#!/usr/bin/env python3
"""
Twist to CARLA Direct Controller
ROS2 Twist 메시지를 받아서 CARLA Python API로 직접 차량 제어
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import carla
import sys
import math


class TwistCarlaController(Node):
    def __init__(self):
        super().__init__('twist_carla_controller')
        
        # 파라미터 선언
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('role_name', 'ego_vehicle')
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('max_steering_angle', 0.7)  # 라디안
        self.declare_parameter('max_speed', 10.0)  # m/s
        self.declare_parameter('wheelbase', 2.87)  # m
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        # 파라미터 가져오기
        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        self.role_name = self.get_parameter('role_name').value
        timeout = self.get_parameter('timeout').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value
        self.wheelbase = self.get_parameter('wheelbase').value
        update_rate = self.get_parameter('update_rate').value
        
        # CARLA 연결
        self.get_logger().info(f'Connecting to CARLA at {host}:{port}...')
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(timeout)
            self.world = self.client.get_world()
            self.get_logger().info(f'✓ Connected to CARLA: {self.world.get_map().name}')
        except Exception as e:
            self.get_logger().error(f'✗ Failed to connect to CARLA: {e}')
            self.get_logger().error('Make sure CARLA is running!')
            sys.exit(1)
        
        # Ego vehicle 찾기
        self.vehicle = None
        self.find_ego_vehicle()
        
        if self.vehicle is None:
            self.get_logger().error(f'✗ No vehicle with role_name "{self.role_name}" found!')
            self.get_logger().info('Spawning new ego_vehicle...')
            self.spawn_ego_vehicle()
        
        if self.vehicle is None:
            self.get_logger().error('Failed to spawn vehicle. Exiting.')
            sys.exit(1)
        
        # Vehicle 설정
        self.vehicle.set_autopilot(False)
        self.vehicle.set_simulate_physics(True)
        self.get_logger().info(f'✓ Found ego_vehicle: {self.vehicle.id}')
        self.get_logger().info(f'  Type: {self.vehicle.type_id}')
        self.get_logger().info(f'  Location: {self.vehicle.get_location()}')
        
        # 마지막 제어 명령 저장
        self.last_twist = Twist()
        self.msg_count = 0
        
        # ROS2 Subscriber 생성
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        
        # 타이머로 주기적으로 제어 명령 전송 (제어 안정성 향상)
        self.timer = self.create_timer(1.0 / update_rate, self.control_timer_callback)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Twist to CARLA Direct Controller Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribing to: /cmd_vel')
        self.get_logger().info('Ready to receive Twist commands!')
        self.get_logger().info('='*60)
        
    def find_ego_vehicle(self):
        """role_name이 ego_vehicle인 차량 찾기"""
        for actor in self.world.get_actors().filter('vehicle.*'):
            if 'role_name' in actor.attributes:
                if actor.attributes['role_name'] == self.role_name:
                    self.vehicle = actor
                    return
    
    def spawn_ego_vehicle(self):
        """Ego vehicle 자동 스폰"""
        try:
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
            vehicle_bp.set_attribute('role_name', self.role_name)
            
            spawn_points = self.world.get_map().get_spawn_points()
            if len(spawn_points) == 0:
                self.get_logger().error('No spawn points available!')
                return
            
            spawn_point = spawn_points[0]
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            self.get_logger().info(f'✓ Spawned ego_vehicle at {spawn_point.location}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to spawn vehicle: {e}')
    
    def twist_callback(self, msg):
        """Twist 메시지 수신 콜백"""
        self.last_twist = msg
    
    def control_timer_callback(self):
        """주기적으로 제어 명령 적용"""
        if self.vehicle is None:
            self.get_logger().warn('Vehicle not found, searching again...')
            self.find_ego_vehicle()
            return
        
        msg = self.last_twist
        
        try:
            # CARLA VehicleControl 생성
            control = carla.VehicleControl()
            
            # Linear velocity (x) -> Throttle/Brake/Reverse
            desired_speed = msg.linear.x * 100
            
            if desired_speed > 0.01:  # 전진
                # Throttle 계산 (0.0 ~ 1.0)
                control.throttle = min(abs(desired_speed) / self.max_speed, 1.0)
                control.brake = 0.0
                control.reverse = False
                
            elif desired_speed < -0.01:  # 후진
                control.throttle = max(abs(desired_speed) / self.max_speed, 1.0)
                control.brake = 0.0
                control.reverse = True
                
            else:  # 정지
                control.throttle = 0.0
                control.brake = 0.5
                control.reverse = False
            
            # Angular velocity (z) -> Steering
            angular_vel = -(msg.angular.z)*10
            
            if abs(desired_speed) > 0.01:
                # Ackermann 조향 모델
                # 회전 반경: R = v / ω
                if abs(angular_vel) > 0.001:
                    turning_radius = desired_speed / angular_vel
                    # 조향각: δ = atan(L / R)
                    steering_angle = math.atan(self.wheelbase / turning_radius)
                    # 정규화 [-1, 1]
                    control.steer = max(-1.0, min(1.0, steering_angle / self.max_steering))
                else:
                    control.steer = 0.0
            else:
                # 제자리 회전
                if abs(angular_vel) > 0.01:
                    control.steer = 1.0 if angular_vel > 0 else -1.0
                else:
                    control.steer = 0.0
            
            # Hand brake 및 기타 설정
            control.hand_brake = False
            control.manual_gear_shift = False
            
            # 차량에 제어 적용
            self.vehicle.apply_control(control)
            
            # 주기적 로그 출력
            self.msg_count += 1
            if self.msg_count % 100 == 0:  # 2초마다 (50Hz 기준)
                vel = self.vehicle.get_velocity()
                speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                self.get_logger().info(
                    f'Twist: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f} | '
                    f'Control: throttle={control.throttle:.2f}, steer={control.steer:.2f}, '
                    f'brake={control.brake:.2f}, reverse={control.reverse} | '
                    f'Speed: {speed:.2f} m/s'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error applying control: {e}')
            self.vehicle = None
    
    def destroy(self):
        """종료 시 정리"""
        if self.vehicle is not None:
            # 차량 정지
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = True
            self.vehicle.apply_control(control)
            self.get_logger().info('Vehicle stopped')


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = TwistCarlaController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
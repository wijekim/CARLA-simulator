import rclpy
from rclpy.node import Node
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # --- 제어 파라미터 (CARLA 환경에 최적화) ---
        self.k = 0.15            # Stanley Gain (0.3 ~ 1.0 사이 추천)
        self.k_soft = 1.5       # 분모 안정화 상수
        self.wheel_base = 2.7   # 차량 축거 (m)
        self.target_v = 0.5    # 목표 선속도 (m/s)
        self.max_steer = 0.4    # 최대 조향각 (약 30도)

        # --- 상태 변수 ---
        self.current_pose = None
        self.path = None

        # --- 발행 및 구독 설정 ---
        self.pose_sub = self.create_subscription(PoseStamped, '/ego_vehicle/pose', self.pose_callback, 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 20Hz 주기로 제어 루프 실행
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Stanley Controller for CARLA has been started.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def path_callback(self, msg):
        if len(msg.poses) > 0:
            self.path = msg

    def get_yaw(self, pose_msg):
        """쿼터니언 데이터를 받아 직접 Yaw(Heading) 값을 계산합니다."""
        q = pose_msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """각도를 -pi ~ pi 범위로 정규화합니다."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self):
        # 데이터가 부족하면 실행하지 않음
        if self.current_pose is None or self.path is None or len(self.path.poses) < 2:
            return

        # 1. 현재 차량 정보 추출
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        curr_yaw = self.get_yaw(self.current_pose)
        
        # 2. 전륜(Front Axle) 위치 계산 (Center 기준 약 1.35m 앞)
        front_x = curr_x + (self.wheel_base / 2.0) * math.cos(curr_yaw)
        front_y = curr_y + (self.wheel_base / 2.0) * math.sin(curr_yaw)

        # 3. 전방의 가장 가까운 타겟 포인트 찾기
        min_dist = float('inf')
        target_idx = 0
        for i, pose in enumerate(self.path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((front_x - px)**2 + (front_y - py)**2)
            
            # 내적을 통한 전방 확인 (차량이 지나친 점은 무시)
            target_vec_x = px - curr_x
            target_vec_y = py - curr_y
            dot = target_vec_x * math.cos(curr_yaw) + target_vec_y * math.sin(curr_yaw)
            
            if dot > 0 and dist < min_dist:
                min_dist = dist
                target_idx = i

        target_pose = self.path.poses[target_idx]
        
        # 4. 경로 방향(Path Yaw) 계산
        # orientation이 부정확할 경우를 대비해 인접 점으로 방향 계산
        if target_idx < len(self.path.poses) - 1:
            p1 = self.path.poses[target_idx].pose.position
            p2 = self.path.poses[target_idx + 1].pose.position
            path_yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
        else:
            path_yaw = self.get_yaw(target_pose)

        # 5. 오차 계산 (Cross-track Error)
        dx = target_pose.pose.position.x - front_x
        dy = target_pose.pose.position.y - front_y
        # 경로 법선 벡터와의 외적 개념을 이용한 좌(+)/우(-) 판별
        cte = dy * math.cos(path_yaw) - dx * math.sin(path_yaw)

        # 6. 헤딩 오차 계산
        theta_e = self.normalize_angle(path_yaw - curr_yaw)

        # 7. Stanley 제어 공식 적용
        v_current = max(self.target_v, 0.5) 
        theta_cross = math.atan2(self.k * cte, v_current + self.k_soft)
        
        # [핵심 수정] 제자리 회전 방지: 부호가 반대일 경우 아래의 마이너스(-)를 제거하거나 붙이세요.
        # 현재 CARLA 로그 상 음수 각속도가 양수 조향으로 변환되므로 전체에 -를 붙여 반전시킵니다.
        steer_angle = theta_e + theta_cross
        
        # 조향각 제한
        steer_angle = np.clip(steer_angle, -self.max_steer, self.max_steer)

        # 8. 명령 발행
        twist = Twist()
        twist.linear.x = float(self.target_v)
        # CARLA Twist 인터페이스에 맞춰 조향각 직접 입력
        twist.angular.z = float(steer_angle) 
        
        self.cmd_pub.publish(twist)

        # 디버깅 로그: 이 값을 통해 오차가 줄어드는지 확인하세요.
        self.get_logger().info(f"CTE: {cte:.2f}, Steer: {steer_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#goal stop
# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math

# from geometry_msgs.msg import PoseStamped, Twist
# from nav_msgs.msg import Path

# class StanleyController(Node):
#     def __init__(self):
#         super().__init__('stanley_controller')

#         # --- 제어 파라미터 (튜닝 포인트) ---
#         self.k = 0.15           # 게인을 더 낮춤 (흔들림 방지)
#         self.k_soft = 1.5       # 분모 상수를 키워 저속에서 안정화
#         self.wheel_base = 2.7
#         self.target_v = 0.5     # 현재 저속 설정 유지
#         self.max_steer = 0.4
#         self.goal_tolerance = 1.0 # 1m 이내 도착 시 정지

#         self.current_pose = None
#         self.path = None
#         self.goal_reached = False

#         self.pose_sub = self.create_subscription(PoseStamped, '/ego_vehicle/pose', self.pose_callback, 10)
#         self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.timer = self.create_timer(0.05, self.control_loop)
#         self.get_logger().info('Stanley Controller: Stability & Goal-Stop Mode enabled.')

#     def pose_callback(self, msg): self.current_pose = msg
#     def path_callback(self, msg):
#         if len(msg.poses) > 0:
#             self.path = msg
#             self.goal_reached = False # 새 경로 수신 시 초기화

#     def get_yaw(self, pose_msg):
#         q = pose_msg.pose.orientation
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)
    
#     def normalize_angle(self, angle):
#         return math.atan2(math.sin(angle), math.cos(angle))

#     def control_loop(self):
#         if self.current_pose is None or self.path is None or len(self.path.poses) < 2:
#             return

#         curr_x = self.current_pose.pose.position.x
#         curr_y = self.current_pose.pose.position.y
#         curr_yaw = self.get_yaw(self.current_pose)
        
#         # --- [추가] 목적지 정지 로직 ---
#         goal_pos = self.path.poses[-1].pose.position
#         dist_to_goal = math.sqrt((curr_x - goal_pos.x)**2 + (curr_y - goal_pos.y)**2)

#         if dist_to_goal < self.goal_tolerance or self.goal_reached:
#             if not self.goal_reached:
#                 self.get_logger().info('--- Goal Reached! ---')
#                 self.goal_reached = True
#             self.stop_vehicle()
#             return

#         # 1. 전륜 위치 계산
#         front_x = curr_x + (self.wheel_base / 2.0) * math.cos(curr_yaw)
#         front_y = curr_y + (self.wheel_base / 2.0) * math.sin(curr_yaw)

#         # 2. 타겟 포인트 찾기 (가장 가까운 점이 아니라 약간 앞을 보게 설정 가능)
#         min_dist = float('inf')
#         target_idx = 0
#         for i, pose in enumerate(self.path.poses):
#             px, py = pose.pose.position.x, pose.pose.position.y
#             dist = math.sqrt((front_x - px)**2 + (front_y - py)**2)
            
#             # 전방 확인
#             dot = (px - curr_x) * math.cos(curr_yaw) + (py - curr_y) * math.sin(curr_yaw)
#             if dot > 0 and dist < min_dist:
#                 min_dist = dist
#                 target_idx = i

#         target_pose = self.path.poses[target_idx]
        
#         # 3. 경로 방향 계산 (부드러운 조향을 위해 다음 점과의 각도 계산)
#         if target_idx < len(self.path.poses) - 1:
#             p1, p2 = self.path.poses[target_idx].pose.position, self.path.poses[target_idx+1].pose.position
#             path_yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
#         else:
#             path_yaw = self.get_yaw(target_pose)

#         # 4. 오차 계산 (CTE)
#         dx, dy = target_pose.pose.position.x - front_x, target_pose.pose.position.y - front_y
#         cte = dy * math.cos(path_yaw) - dx * math.sin(path_yaw)

#         # 5. 헤딩 오차
#         theta_e = self.normalize_angle(path_yaw - curr_yaw)

#         # 6. Stanley 제어 (흔들림 방지를 위해 k 낮춤 + k_soft 키움)
#         v_current = max(self.target_v, 0.5) 
#         theta_cross = math.atan2(self.k * cte, v_current + self.k_soft)
        
#         # 지난번 성공했던 부호 방향 유지
#         steer_angle = theta_e + theta_cross
#         steer_angle = np.clip(steer_angle, -self.max_steer, self.max_steer)

#         # 7. 명령 발행
#         self.publish_cmd(self.target_v, steer_angle)

#     def stop_vehicle(self):
#         self.publish_cmd(0.0, 0.0)

#     def publish_cmd(self, v, steer):
#         twist = Twist()
#         twist.linear.x = float(v)
#         twist.angular.z = float(steer)
#         self.cmd_pub.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = StanleyController()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally: node.destroy_node(); rclpy.shutdown()

# if __name__ == '__main__': main()
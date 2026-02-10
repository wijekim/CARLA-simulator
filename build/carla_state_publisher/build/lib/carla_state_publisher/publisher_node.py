import carla
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseArray, Pose, Quaternion
import math
import copy

class CarlaStatePublisher(Node):
    def __init__(self):
        super().__init__('carla_state_publisher')
        
        # 1. ROS2 Publisher & Broadcaster 설정
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, 'ego_vehicle/pose', 10)
        self.trajectory_pub = self.create_publisher(PoseArray, 'ego_vehicle/trajectory', 10)
        
        # 흔적 저장을 위한 변수
        self.poses_list = [] 
        self.last_recorded_loc = None # 중복 기록 방지용
        
        # 2. CARLA 연결
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.get_logger().info("Connected to CARLA Server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        self.ego_vehicle = None
        self.find_ego_vehicle()
        
        # 4. 주기적 실행 (20Hz)
        self.timer = self.create_timer(0.05, self.update_step)
        
        
        self.flag = True

        self.initialpose_loc = carla.Location(x=0.0, y=0.0, z=0.0)
        self.initialpose_rot = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        


    def find_ego_vehicle(self):
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') in ['ego_vehicle', 'hero']:
                self.ego_vehicle = actor
                self.get_logger().info(f"Found Ego Vehicle: {actor.id}")
                break

    def update_step(self):
        if not self.ego_vehicle:
            self.find_ego_vehicle()
            return

        
        transform = self.ego_vehicle.get_transform()
        loc = transform.location
        rot = transform.rotation
        now = self.get_clock().now().to_msg()

        if(self.flag):
            #self.initialpose_loc = transform.location
            #self.initialpose_rot = transform.rotation

            self.initialpose_loc = self.ego_vehicle.get_transform().location
            self.initialpose_rot = self.ego_vehicle.get_transform().rotation
            self.flag = False
            print(self.flag)
            print(self.initialpose_loc.x,self.initialpose_loc.y,self.initialpose_loc.z)

        print(self.initialpose_loc.x,self.initialpose_loc.y,self.initialpose_loc.z)
        print(self.initialpose_rot.roll,self.initialpose_rot.pitch,self.initialpose_rot.yaw)
        # loc.x= loc.x-self.initialpose_loc.x
        # loc.y= loc.y-self.initialpose_loc.y
        # loc.z= loc.z-self.initialpose_loc.z
        
        # rot.roll = rot.roll - self.initialpose_rot.roll
        # rot.pitch = rot.pitch - self.initialpose_rot.pitch
        # rot.yaw= rot.yaw - self.initialpose_rot.yaw

        # --- 1. TF 및 단일 Pose용 쿼터니언 계산 ---
        q_msg = self.euler_to_quaternion(
            math.radians(-rot.roll), 
            math.radians(-rot.pitch/2), 
            math.radians(-rot.yaw)
        )

        # --- 2. TF 발행 (map -> ego_vehicle) ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'ego_vehicle'
        t.transform.translation.x = loc.x
        t.transform.translation.y = -loc.y
        t.transform.translation.z = loc.z
        t.transform.rotation = q_msg
        self.br.sendTransform(t)

        # --- 3. 실시간 Pose 발행 ---
        pose_stamped = PoseStamped()
        pose_stamped.header = t.header
        pose_stamped.pose.position.x = t.transform.translation.x
        pose_stamped.pose.position.y = t.transform.translation.y
        pose_stamped.pose.position.z = t.transform.translation.z
        pose_stamped.pose.orientation = q_msg
        self.pose_pub.publish(pose_stamped)

        # --- 4. 궤적(PoseArray) 기록 로직 ---
        # 차량이 정지해 있을 때 데이터가 무한히 쌓이는 것을 방지 (0.2m 이상 이동 시 기록)
        curr_loc = [loc.x, -loc.y, loc.z]
        if self.last_recorded_loc is None or self.get_distance(curr_loc, self.last_recorded_loc) > 0.2:
            new_pose = Pose()
            new_pose.position.x = curr_loc[0]
            new_pose.position.y = curr_loc[1]
            new_pose.position.z = curr_loc[2]
            new_pose.orientation = q_msg
            
            self.poses_list.append(new_pose)
            self.last_recorded_loc = curr_loc

        # 누적된 모든 포즈 발행
        path_msg = PoseArray()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'map'
        path_msg.poses = self.poses_list
        self.trajectory_pub.publish(path_msg)

    def get_distance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
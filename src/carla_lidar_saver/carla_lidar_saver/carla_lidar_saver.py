import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os

class LidarSaver(Node):
    def __init__(self):
        super().__init__('lidar_to_ply_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.listener_callback,
            10)
        
        self.save_path = './saved_ply'
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            
        self.count = 0
        self.get_logger().info('LiDAR to PLY Saver Node has started.')

    def listener_callback(self, msg):
        # 1. PointCloud2 메시지를 numpy array로 변환 (x, y, z)
        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        if points.shape[0] == 0:
            return

        # 2. Open3D 포인트 클라우드 객체 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 3. 모든 포인트에 흰색 입히기 (N, 3) 형태의 배열 생성
        # Open3D의 색상은 0.0 ~ 1.0 사이의 값을 가집니다. (1, 1, 1은 흰색)
        white_color = np.ones((points.shape[0], 3)) 
        pcd.colors = o3d.utility.Vector3dVector(white_color)

        # 50프레임마다 저장
        if self.count % 1 == 0:
            filename = os.path.join(self.save_path, f"lidar_{self.count:05d}.ply")
            # write_ascii=False로 저장하면 용량이 적고 로딩이 빠릅니다.
            o3d.io.write_point_cloud(filename, pcd)
            self.get_logger().info(f'Saved with white color: {filename}')
        
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = LidarSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
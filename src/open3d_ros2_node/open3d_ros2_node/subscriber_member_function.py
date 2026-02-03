import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',  #topic name
            self.listener_callback,
            10)
        
        # 2. Open3D Visualizer 설정
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Open3D ROS2 LiDAR', width=800, height=600)

        opt = self.vis.get_render_option()
        opt.point_size = 1.5           # 점 크기를 작게 조절 (네모 느낌 감소)
        #opt.background_color = np.array([0.0, 0.0, 0.0]) 
        #opt.light_on = False           

        self.pcd = o3d.geometry.PointCloud()
        self.is_first_msg = True

    def listener_callback(self, msg):

        #3D
        # 1. 포인트 데이터 변환 (기존과 동일)
        points = [[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
        if not points: return
        self.pcd.points = o3d.utility.Vector3dVector(np.array(points))

        if self.is_first_msg:
            self.vis.add_geometry(self.pcd)
            
            # 2. 시점 제어기 가져오기
            ctr = self.vis.get_view_control()
            
            # ---------------------------------------------------------
            # A. 탑뷰 (위에서 아래로)
            # ctr.set_front([0, 0, -1])  # 카메라가 바라보는 방향 (Z축 아래로)
            # ctr.set_up([0, -1, 0])     # 카메라의 위쪽 방향 설정
            # ctr.set_lookat([0, 0, 0])  # 카메라가 집중할 초점
            # ctr.set_zoom(0.5)          # 숫자가 작을수록 멀리 보임
             # ---------------------------------------------------------
            # B. 쿼터뷰/사선뷰 (대각선 위에서)
            ctr.set_front([-1.0, 0.0, 1.0])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat([0, 0, 0])
            ctr.set_zoom(0.3)
            # ---------------------------------------------------------
            
            self.is_first_msg = False
        
        # 3. 화면 갱신
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vis.destroy_window()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
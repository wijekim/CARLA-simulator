import open3d as o3d
import os
import glob
import numpy as np

class LidarVisualizer:
    def __init__(self, directory):
        self.ply_files = sorted(glob.glob(os.path.join(directory, "*.ply")))
        if not self.ply_files:
            print(f"Error: No .ply files found in {directory}")
            return

        self.index = 0
        self.total = len(self.ply_files)
        print(f"Total {self.total} files found.")
        print("Controls: [→]: Next, [←]: Previous, [Q/ESC]: Exit")

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="CARLA LiDAR Fixed View", width=1280, height=720)

        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        opt.point_size = 1.0

        self.pcd = o3d.io.read_point_cloud(self.ply_files[self.index])
        self.vis.add_geometry(self.pcd)

        # ---------------------------------------------------------
        # 시점 고정 설정 (카메라 위치 조정)
        # ---------------------------------------------------------
        view_ctl = self.vis.get_view_control()
        # 이미지를 참고하여 비스듬한 후방 상단 뷰로 초기화
        # front: 카메라가 바라보는 정면 벡터, lookat: 초점, up: 위쪽 방향, zoom: 확대/축소
        view_ctl.set_front([-0.8, 0.0, 0.5])  # 대각선 뒤쪽 상단에서 바라봄
        view_ctl.set_lookat([0, 0, 0])        # 차량(중심)을 바라봄
        view_ctl.set_up([0, 0, 1])            # Z축이 위쪽
        view_ctl.set_zoom(0.3)                # 숫자가 작을수록 크게 보임
        # ---------------------------------------------------------

        self.vis.register_key_callback(262, self.load_next)
        self.vis.register_key_callback(263, self.load_prev)

        self.vis.run()
        self.vis.destroy_window()

    def update_view(self):
        # 새로운 데이터 로드
        new_pcd = o3d.io.read_point_cloud(self.ply_files[self.index])
        
        # 기존 geometry 데이터 교체
        self.pcd.points = new_pcd.points
        self.pcd.colors = new_pcd.colors
        
        # 화면 갱신
        self.vis.update_geometry(self.pcd)
        self.vis.set_full_screen(False) # 가끔 필요한 화면 리프레시 트리거
        print(f"[{self.index + 1}/{self.total}] Visualizing: {os.path.basename(self.ply_files[self.index])}")

    def load_next(self, vis):
        self.index = (self.index + 1) % self.total
        self.update_view()
        return False

    def load_prev(self, vis):
        self.index = (self.index - 1) % self.total
        self.update_view()
        return False

def main(args=None):
    PATH = "/home/airlab/ros2_ws/saved_ply" 
    if not os.path.exists(PATH):
        print(f"Directory not found: {PATH}")
        return
    
    LidarVisualizer(PATH)

if __name__ == "__main__":
    main()
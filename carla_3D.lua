include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "hero",           -- [중요] IMU와 라이다가 붙어있는 기준 프레임
  published_frame = "hero",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false, 
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,               -- 3D에서는 0으로 설정
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 1,              -- [중요] PointCloud2를 쓰기 위해 1로 설정
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 3D 빌더 활성화 (2D는 반드시 false)
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8

-- [주의] TRAJECTORY_BUILDER_3D 아래에는 use_imu_data 키가 없습니다. 
-- 정의하면 "Key used wrong number of times" 에러가 납니다.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 1.0
TRAJECTORY_BUILDER_3D.max_range = 80.0

-- 3D SLAM 성능을 위한 Voxel 필터 설정
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 50.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 80.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5e2  -- 기존보다 10배 강화
return options
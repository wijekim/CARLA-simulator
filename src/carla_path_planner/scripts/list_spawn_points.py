#!/usr/bin/env python3
"""
CARLA Spawn Point 조회 스크립트

CARLA 맵에 미리 정의되어 있는 spawn point들의 정보를 출력합니다.
각 맵마다 다른 개수와 위치의 spawn point를 가지고 있습니다.

사용법:
    python3 list_spawn_points.py
"""

import carla
import sys


def main():
    # CARLA 연결
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()
        
        print(f"\n{'='*80}")
        print(f"CARLA Map: {carla_map.name}")
        print(f"{'='*80}\n")
        
        # Spawn points 가져오기
        spawn_points = carla_map.get_spawn_points()
        
        print(f"Total spawn points: {len(spawn_points)}")
        print(f"\n참고: 맵별 spawn point 개수는 다릅니다.")
        print(f"  - Town01: ~80개")
        print(f"  - Town02: ~80개") 
        print(f"  - Town03: ~240개")
        print(f"  - Town04: ~380개")
        print(f"  - Town05: ~240개")
        print(f"  등등...\n")
        
        print(f"{'ID':<5} {'X':>10} {'Y':>10} {'Z':>10} {'Yaw':>10} "
              f"{'Lane':<8} {'Road':<8} {'ROS2_X':>10} {'ROS2_Y':>10}")
        print(f"{'-'*100}")
        
        for i, spawn_point in enumerate(spawn_points):
            loc = spawn_point.location
            rot = spawn_point.rotation
            
            # ROS2 좌표계 변환
            ros_x = loc.x
            ros_y = -loc.y
            
            # 해당 spawn point의 waypoint 정보 가져오기
            try:
                waypoint = carla_map.get_waypoint(
                    loc, 
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving
                )
                lane_id = waypoint.lane_id if waypoint else "N/A"
                road_id = waypoint.road_id if waypoint else "N/A"
            except:
                lane_id = "Error"
                road_id = "Error"
            
            print(f"{i:<5} {loc.x:>10.2f} {loc.y:>10.2f} {loc.z:>10.2f} "
                  f"{rot.yaw:>10.2f} {str(lane_id):<8} {str(road_id):<8} "
                  f"{ros_x:>10.2f} {ros_y:>10.2f}")
        
        print(f"\n{'='*100}")
        print(f"좌표계 정보:")
        print(f"  CARLA: X(forward), Y(right), Z(up) - Left-handed")
        print(f"  ROS2:  X(forward), Y(left),  Z(up) - Right-handed")
        print(f"  변환:  ROS2_X = CARLA_X, ROS2_Y = -CARLA_Y")
        print(f"\nLane/Road 정보:")
        print(f"  Lane ID: 차선 번호 (양수=오른쪽 차선, 음수=왼쪽 차선)")
        print(f"  Road ID: 도로 구간 번호")
        print(f"  Driving: 주행 가능한 차선만 표시")
        print(f"{'='*100}\n")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        print("\nCARLA 서버가 실행 중인지 확인하세요:")
        print("  cd /path/to/CARLA_0.10.0")
        print("  ./CarlaUE5.sh")
        sys.exit(1)


if __name__ == '__main__':
    main()

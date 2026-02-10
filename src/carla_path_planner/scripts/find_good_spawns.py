#!/usr/bin/env python3
"""
CARLA Spawn Point 선택 도우미

경로 계획에 적합한 spawn point를 찾아줍니다.
- Driving lane에 있는 spawn point만 표시
- 두 spawn point 간 경로 계획 가능 여부 테스트
"""

import carla
import sys
import math


def calculate_distance(loc1, loc2):
    """두 위치 간 직선 거리 계산"""
    return math.sqrt(
        (loc1.x - loc2.x)**2 + 
        (loc1.y - loc2.y)**2 + 
        (loc1.z - loc2.z)**2
    )


def test_route_planning(carla_map, grp, start_id, goal_id, spawn_points):
    """두 spawn point 간 경로 계획 테스트"""
    start_loc = spawn_points[start_id].location
    goal_loc = spawn_points[goal_id].location
    
    # Waypoint로 변환 (Driving lane만)
    start_wp = carla_map.get_waypoint(
        start_loc,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    goal_wp = carla_map.get_waypoint(
        goal_loc,
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )
    
    if start_wp is None or goal_wp is None:
        return False, 0, "Failed to project to road"
    
    # 경로 계획
    try:
        route = grp.trace_route(start_wp.transform.location, goal_wp.transform.location)
        if route and len(route) > 0:
            return True, len(route), "OK"
        else:
            return False, 0, "No route found"
    except Exception as e:
        return False, 0, f"Error: {str(e)[:50]}"


def main():
    # CARLA 연결
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()
        
        print(f"\n{'='*80}")
        print(f"CARLA Spawn Point 선택 도우미")
        print(f"Map: {carla_map.name}")
        print(f"{'='*80}\n")
        
    except Exception as e:
        print(f"Error connecting to CARLA: {e}", file=sys.stderr)
        sys.exit(1)
    
    # GlobalRoutePlanner 초기화
    try:
        from agents.navigation.global_route_planner import GlobalRoutePlanner
        grp = GlobalRoutePlanner(carla_map, 2.0)
        print("GlobalRoutePlanner initialized ✓\n")
    except ImportError as e:
        print(f"Warning: GlobalRoutePlanner not available: {e}")
        print("Route planning test will be skipped.\n")
        grp = None
    
    # Spawn points 가져오기
    spawn_points = carla_map.get_spawn_points()
    
    # Driving lane에 있는 spawn point만 필터링
    valid_spawns = []
    
    print("Analyzing spawn points...")
    for i, sp in enumerate(spawn_points):
        wp = carla_map.get_waypoint(
            sp.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        if wp is not None:
            # Spawn point와 waypoint 간 거리
            dist = calculate_distance(sp.location, wp.transform.location)
            
            valid_spawns.append({
                'id': i,
                'spawn': sp,
                'waypoint': wp,
                'distance_to_road': dist
            })
    
    print(f"\nTotal spawn points: {len(spawn_points)}")
    print(f"Valid driving lane spawns: {len(valid_spawns)}")
    print(f"\nRecommended spawn points for path planning:\n")
    
    print(f"{'ID':<5} {'X':>10} {'Y':>10} {'Lane':<6} {'Road':<6} {'Dist':>8}")
    print(f"{'-'*50}")
    
    for item in valid_spawns[:50]:  # 처음 50개만 표시
        sp = item['spawn']
        wp = item['waypoint']
        dist = item['distance_to_road']
        
        print(f"{item['id']:<5} {sp.location.x:>10.2f} {sp.location.y:>10.2f} "
              f"{wp.lane_id:<6} {wp.road_id:<6} {dist:>7.2f}m")
    
    if len(valid_spawns) > 50:
        print(f"... and {len(valid_spawns) - 50} more")
    
    # 경로 테스트 (옵션)
    if grp is not None and len(valid_spawns) >= 2:
        print(f"\n{'='*80}")
        print("경로 계획 테스트")
        print(f"{'='*80}\n")
        
        # 몇 가지 경로 조합 테스트
        test_pairs = [
            (valid_spawns[0]['id'], valid_spawns[min(10, len(valid_spawns)-1)]['id']),
            (valid_spawns[0]['id'], valid_spawns[min(20, len(valid_spawns)-1)]['id']),
            (valid_spawns[0]['id'], valid_spawns[-1]['id']),
        ]
        
        print(f"{'From':<6} {'To':<6} {'Status':<12} {'Waypoints':<10} {'Note'}")
        print(f"{'-'*60}")
        
        for start_id, goal_id in test_pairs:
            if start_id != goal_id:
                success, waypoints, note = test_route_planning(
                    carla_map, grp, start_id, goal_id, spawn_points
                )
                status = "✓ SUCCESS" if success else "✗ FAILED"
                print(f"{start_id:<6} {goal_id:<6} {status:<12} {waypoints:<10} {note}")
    
    # 사용 예시
    print(f"\n{'='*80}")
    print("사용 예시:")
    print(f"{'='*80}\n")
    
    if len(valid_spawns) >= 2:
        example_start = valid_spawns[0]['id']
        example_goal = valid_spawns[min(20, len(valid_spawns)-1)]['id']
        
        print(f"ros2 launch carla_path_planner path_planner.launch.py \\")
        print(f"    start_spawn_id:={example_start} \\")
        print(f"    goal_spawn_id:={example_goal}")
        
        if len(valid_spawns) >= 4:
            mid1 = valid_spawns[min(5, len(valid_spawns)-1)]['id']
            mid2 = valid_spawns[min(10, len(valid_spawns)-1)]['id']
            print(f"\n# 경유점 포함:")
            print(f"ros2 launch carla_path_planner path_planner.launch.py \\")
            print(f"    start_spawn_id:={example_start} \\")
            print(f"    goal_spawn_id:={example_goal} \\")
            print(f"    waypoint_ids:=\"{mid1},{mid2}\"")
    
    print()


if __name__ == '__main__':
    main()

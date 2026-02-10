#!/bin/bash
# CARLA Path Planner 실행 예제 스크립트

echo "CARLA Path Planner 예제 실행 스크립트"
echo "======================================"
echo ""

# ROS2 환경 소싱 확인
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 환경이 설정되지 않았습니다."
    echo "다음 명령을 실행하세요:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "사용 가능한 예제:"
echo ""
echo "1. 기본 경로 계획 (spawn 0 → 50)"
echo "2. 커스텀 경로 (spawn 10 → 100)"
echo "3. 경유점 포함 경로 (spawn 0 → 25 → 50 → 100)"
echo "4. Spawn point 목록 보기"
echo ""
read -p "선택 (1-4): " choice

case $choice in
    1)
        echo "기본 경로 계획 실행..."
        ros2 launch carla_path_planner path_planner.launch.py
        ;;
    2)
        echo "커스텀 경로 실행..."
        ros2 launch carla_path_planner path_planner.launch.py \
            start_spawn_id:=10 \
            goal_spawn_id:=100
        ;;
    3)
        echo "경유점 포함 경로 실행..."
        ros2 launch carla_path_planner path_planner.launch.py \
            start_spawn_id:=0 \
            goal_spawn_id:=100 \
            waypoint_ids:="25,50,75"
        ;;
    4)
        echo "Spawn point 목록 조회..."
        python3 $(ros2 pkg prefix carla_path_planner)/../../src/carla_path_planner/scripts/list_spawn_points.py
        ;;
    *)
        echo "잘못된 선택입니다."
        exit 1
        ;;
esac

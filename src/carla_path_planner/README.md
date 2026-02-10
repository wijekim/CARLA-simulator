# CARLA Path Planner for ROS2 Nav2

CARLA 0.10.0과 ROS2 Jazzy를 연동하여 경로 계획을 수행하는 패키지입니다. Nav2와 호환되는 표준 ROS 메시지(`nav_msgs/msg/Path`)를 사용합니다.

## 기능

- **기존 Spawn Point 사용**: CARLA 맵에 미리 정의된 spawn point의 ID를 사용하여 출발점/도착점/경유점 지정
- **도로 기반 경로 계획**: CARLA의 GlobalRoutePlanner를 사용한 도로 네트워크 상의 최단 경로 계산
- **자동 좌표 변환**: CARLA 좌표계(left-handed)를 ROS2 좌표계(right-handed)로 자동 변환
- **Nav2 호환**: 표준 ROS `nav_msgs/msg/Path` 메시지를 사용하여 `/plan` 토픽으로 경로 발행
- **경유점 지원**: 여러 개의 중간 경유점을 통과하는 경로 계획 가능

## 요구사항

### 시스템 요구사항
- Ubuntu 24.04
- ROS2 Jazzy
- CARLA 0.10.0
- Python 3.10+

### Python 패키지
```bash
pip install carla==0.10.0
```

### ROS2 패키지
```bash
sudo apt install ros-jazzy-nav-msgs ros-jazzy-geometry-msgs
```

## 설치

1. ROS2 워크스페이스로 이동:
```bash
cd ~/ros2_ws/src
```

2. 패키지 복사:
```bash
cp -r carla_path_planner .
```

3. 빌드:
```bash
cd ~/ros2_ws
colcon build --packages-select carla_path_planner
source install/setup.bash
```

## 사용법

### 0. CARLA PythonAPI 환경 설정 (중요!)

CARLA의 GlobalRoutePlanner를 사용하려면 CARLA PythonAPI가 PYTHONPATH에 있어야 합니다.

```bash
# CARLA 설치 경로 설정 (본인의 경로에 맞게 수정)
export CARLA_ROOT=$HOME/carla/CARLA_0.10.0

# 환경 설정 스크립트 실행
cd ~/ros2_ws/src/carla_path_planner
source scripts/setup_carla_env.sh
```

또는 수동으로 설정:

```bash
export CARLA_ROOT=$HOME/carla/CARLA_0.10.0
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/agents:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$PYTHONPATH
```

**영구 설정** (~/.bashrc에 추가):
```bash
echo 'export CARLA_ROOT=$HOME/carla/CARLA_0.10.0' >> ~/.bashrc
echo 'export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH' >> ~/.bashrc
echo 'export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/agents:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

### 1. CARLA 서버 실행

```bash
cd /path/to/CARLA_0.10.0
./CarlaUE5.sh
```

### 2. 패키지 실행

#### 기본 실행 (spawn point 0 → 50):
```bash
ros2 launch carla_path_planner path_planner.launch.py
```

#### 커스텀 spawn point 지정:
```bash
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=10 \
    goal_spawn_id:=100
```

#### 경유점 포함:
```bash
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=0 \
    goal_spawn_id:=100 \
    waypoint_ids:="25,50,75"
```

#### 경로 해상도 조정:
```bash
ros2 launch carla_path_planner path_planner.launch.py \
    path_resolution:=2.0
```

### 3. 발행되는 토픽 확인

```bash
# 토픽 리스트 확인
ros2 topic list

# 경로 데이터 확인
ros2 topic echo /plan
```

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `carla_host` | string | "localhost" | CARLA 서버 주소 |
| `carla_port` | int | 2000 | CARLA 서버 포트 |
| `carla_timeout` | float | 10.0 | CARLA 연결 타임아웃 (초) |
| `start_spawn_id` | int | 0 | 출발점 spawn point ID |
| `goal_spawn_id` | int | 1 | 도착점 spawn point ID |
| `waypoint_ids` | string | "" | 경유점 spawn point ID 리스트 (쉼표 구분, 예: "10,20,30") |
| `path_resolution` | float | 1.0 | 경로 waypoint 간격 (미터) |

## Spawn Point 확인

**중요**: CARLA의 각 맵에는 미리 정의된 spawn point들이 있습니다. 이 패키지는 새로운 spawn point를 생성하는 것이 아니라, **CARLA 맵에 이미 존재하는 spawn point의 ID를 사용**합니다.

### 맵의 Spawn Point 목록 확인

```bash
# 제공된 스크립트 사용 (모든 spawn point 정보)
python3 scripts/list_spawn_points.py

# 경로 계획에 적합한 spawn point 찾기 (권장!)
python3 scripts/find_good_spawns.py
```

**중요**: 모든 spawn point가 경로 계획에 적합한 것은 아닙니다!
- 일부 spawn point는 주차장, 인도 등 비주행 영역에 있을 수 있습니다
- `find_good_spawns.py`를 사용하면 **주행 가능한 차선(Driving lane)**에 있는 spawn point만 표시됩니다

또는 Python으로 직접 확인:

```python
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
spawn_points = world.get_map().get_spawn_points()

print(f"Total spawn points: {len(spawn_points)}")
for i, sp in enumerate(spawn_points):
    print(f"Spawn {i}: x={sp.location.x:.2f}, y={sp.location.y:.2f}, z={sp.location.z:.2f}")
```

### Spawn Point 특징

- 각 CARLA 맵마다 spawn point 개수가 다릅니다
- Town01: 약 80개, Town02: 약 80개, Town03: 약 240개 등
- Spawn point는 주로 도로 위의 차량이 생성될 수 있는 위치에 배치되어 있습니다
- ID는 0부터 시작합니다

## 좌표계 변환

### CARLA 좌표계
- X: 전방 (Forward)
- Y: 우측 (Right)
- Z: 상방 (Up)
- Left-handed 좌표계

### ROS2 좌표계
- X: 전방 (Forward)
- Y: 좌측 (Left)
- Z: 상방 (Up)
- Right-handed 좌표계

### 변환 공식
```
ROS2_X = CARLA_X
ROS2_Y = -CARLA_Y
ROS2_Z = CARLA_Z
ROS2_Yaw = -CARLA_Yaw
```

## 토픽 인터페이스

### Published Topics

- `/plan` (`nav_msgs/msg/Path`): 계획된 경로
  - `header`: 타임스탬프 및 frame_id ('map')
  - `poses`: PoseStamped 배열 (경로 waypoint)

## 문제 해결

### ImportError: cannot import GlobalRoutePlanner
CARLA PythonAPI가 PYTHONPATH에 없습니다:
```bash
export CARLA_ROOT=$HOME/carla/CARLA_0.10.0  # 본인의 경로로 수정
source scripts/setup_carla_env.sh
```

### 경로가 직선으로 표시됨
GlobalRoutePlanner가 제대로 작동하지 않는 경우입니다:
1. CARLA PythonAPI 경로가 제대로 설정되었는지 확인
2. CARLA 0.10.0 버전이 맞는지 확인
3. 로그에서 "GlobalRoutePlanner initialized" 메시지 확인

### CARLA 연결 실패
```bash
# CARLA 서버가 실행 중인지 확인
ps aux | grep Carla

# 포트 확인
netstat -tuln | grep 2000
```

### Invalid spawn point ID
- CARLA 맵마다 spawn point 개수가 다릅니다
- 위의 "Spawn Point 확인" 스크립트로 유효한 ID 범위를 확인하세요

### 경로 계획 실패
- spawn point가 도로 위에 있는지 확인
- 너무 먼 거리의 경로는 계획이 실패할 수 있습니다
- `path_resolution` 값을 조정해보세요

## 라이선스

MIT License

## 참고 자료

- [CARLA Documentation](https://carla.readthedocs.io/)
- [ROS2 Nav2](https://navigation.ros.org/)
- [nav2_msgs](https://github.com/ros-planning/navigation2/tree/main/nav2_msgs)

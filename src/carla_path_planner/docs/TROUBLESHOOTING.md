# 문제 해결 가이드

## 경로가 직선으로 표시되는 문제

### 증상
- Rviz나 다른 시각화 도구에서 경로가 도로를 따라가지 않고 직선으로 표시됨
- 경로가 건물이나 장애물을 통과함

### 원인
CARLA의 GlobalRoutePlanner가 제대로 작동하지 않는 경우입니다. 주요 원인:

1. **CARLA PythonAPI 경로 문제** (가장 흔함)
2. GlobalRoutePlanner import 실패
3. CARLA 버전 불일치

### 해결 방법

#### 1. CARLA PythonAPI 경로 확인

```bash
# 환경 변수 확인
echo $CARLA_ROOT
# 출력 예: /home/user/carla/CARLA_0.10.0

echo $PYTHONPATH | tr ':' '\n'
# 다음 경로들이 포함되어야 함:
# /home/user/carla/CARLA_0.10.0/PythonAPI/carla
# /home/user/carla/CARLA_0.10.0/PythonAPI/carla/agents
# /home/user/carla/CARLA_0.10.0/PythonAPI
```

#### 2. Python에서 import 테스트

```bash
python3 << EOF
try:
    import carla
    print("✓ carla module OK")
except ImportError as e:
    print("✗ carla module failed:", e)

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    print("✓ GlobalRoutePlanner OK")
except ImportError as e:
    print("✗ GlobalRoutePlanner failed:", e)
    print("  Fix: export PYTHONPATH=\$CARLA_ROOT/PythonAPI/carla/agents:\$PYTHONPATH")
EOF
```

#### 3. 경로 설정 (문제가 있는 경우)

**임시 설정**:
```bash
export CARLA_ROOT=$HOME/carla/CARLA_0.10.0  # 본인의 경로로 수정
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/agents:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$PYTHONPATH

# egg 파일도 추가 (있는 경우)
CARLA_EGG=$(find $CARLA_ROOT/PythonAPI/carla/dist -name "carla-*.egg" | head -1)
export PYTHONPATH=$CARLA_EGG:$PYTHONPATH
```

**영구 설정** (~/.bashrc에 추가):
```bash
cat >> ~/.bashrc << 'EOF'

# CARLA Environment
export CARLA_ROOT=$HOME/carla/CARLA_0.10.0
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/agents:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$PYTHONPATH
EOF

source ~/.bashrc
```

#### 4. 패키지 재빌드

```bash
cd ~/ros2_ws
colcon build --packages-select carla_path_planner --symlink-install
source install/setup.bash
```

#### 5. 노드 실행 및 로그 확인

```bash
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=124 \
    goal_spawn_id:=30
```

**정상 로그**:
```
[INFO] [carla_path_planner]: Connecting to CARLA at localhost:2000...
[INFO] [carla_path_planner]: Connected to CARLA world: Town03
[INFO] [carla_path_planner]: GlobalRoutePlanner initialized with resolution 1.0m
[INFO] [carla_path_planner]: CARLA Path Planner initialized
[INFO] [carla_path_planner]: Using spawn point 124: x=123.45, y=-67.89
[INFO] [carla_path_planner]: Using spawn point 30: x=234.56, y=-78.90
[INFO] [carla_path_planner]: Planning route from spawn 124 to spawn 30
[INFO] [carla_path_planner]: Published path with 150 waypoints
```

**문제 있는 로그**:
```
[ERROR] [carla_path_planner]: Failed to import GlobalRoutePlanner: No module named 'agents'
```

## 기타 문제

### "No route found" 경고

경로 계획이 실패한 경우입니다.

**원인**:
- Spawn point가 도로에서 너무 멀리 떨어져 있음
- 도로 네트워크가 연결되지 않음
- 경유점이 너무 많거나 비현실적인 경로

**해결**:
```bash
# 다른 spawn point 시도
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=0 \
    goal_spawn_id:=50

# path_resolution 조정
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=124 \
    goal_spawn_id:=30 \
    path_resolution:=2.0
```

### CARLA 연결 실패

```bash
# CARLA 서버 실행 확인
ps aux | grep Carla

# 포트 확인
netstat -tuln | grep 2000

# CARLA 재시작
cd $CARLA_ROOT
./CarlaUE5.sh
```

### Python 버전 문제

```bash
# Python 버전 확인 (3.10 이상 권장)
python3 --version

# pip 패키지 확인
pip3 show carla
```

### ROS2 메시지 타입 오류

```bash
# nav_msgs 패키지 설치 확인
ros2 interface show nav_msgs/msg/Path

# 없으면 설치
sudo apt install ros-jazzy-nav-msgs
```

## 검증 체크리스트

패키지가 제대로 작동하는지 확인:

- [ ] CARLA 서버가 실행 중
- [ ] `$CARLA_ROOT` 환경 변수가 설정됨
- [ ] PYTHONPATH에 CARLA PythonAPI 경로가 포함됨
- [ ] `from agents.navigation.global_route_planner import GlobalRoutePlanner` 성공
- [ ] 노드 실행 시 "GlobalRoutePlanner initialized" 로그 출력
- [ ] "Published path with N waypoints" 로그 출력 (N > 10)
- [ ] `/plan` 토픽이 발행됨: `ros2 topic echo /plan`
- [ ] 경로가 도로를 따라감 (Rviz 등에서 확인)

## 추가 디버깅

### 상세 로그 활성화

```bash
ros2 launch carla_path_planner path_planner.launch.py \
    --ros-args --log-level DEBUG
```

### Python 스크립트로 직접 테스트

```python
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

# CARLA 연결
client = carla.Client('localhost', 2000)
world = client.get_world()
carla_map = world.get_map()

# GlobalRoutePlanner 생성
grp = GlobalRoutePlanner(carla_map, 1.0)

# Spawn points 가져오기
spawn_points = carla_map.get_spawn_points()
start = spawn_points[124]
goal = spawn_points[30]

# 경로 계획
route = grp.trace_route(start.location, goal.location)
print(f"Route found with {len(route)} waypoints")

if len(route) > 0:
    print("✓ GlobalRoutePlanner working correctly")
else:
    print("✗ Route planning failed")
```

이 스크립트가 성공하면 CARLA 쪽은 정상이고, ROS2 패키지에 문제가 있는 것입니다.

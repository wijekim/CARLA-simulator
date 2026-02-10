# CARLA Spawn Points 이해하기

## Spawn Point란?

CARLA의 각 맵에는 **미리 정의된 spawn point**들이 있습니다. 이것은 차량이나 보행자를 생성할 수 있는 위치를 나타냅니다.

### 주요 특징

1. **맵에 고정**: 각 CARLA 맵마다 고정된 위치에 spawn point가 배치되어 있습니다
2. **도로 위치**: 대부분의 spawn point는 도로 위에 위치하지만, 일부는 **주차장, 인도, 비주행 영역**에 있을 수 있습니다
3. **0부터 시작**: ID는 0부터 시작하여 순차적으로 증가합니다
4. **맵마다 다름**: 각 맵의 크기와 구조에 따라 spawn point 개수가 다릅니다

**⚠️ 중요**: 모든 spawn point가 경로 계획에 적합한 것은 아닙니다!
- 일부 spawn point는 주행 차선(Driving lane)이 아닌 곳에 위치할 수 있습니다
- 경로 계획 시 자동으로 가장 가까운 주행 차선으로 투영되지만, 너무 멀리 떨어진 경우 예상치 못한 경로가 생성될 수 있습니다

## 맵별 Spawn Point 개수

| 맵 이름 | Spawn Point 개수 (대략) |
|---------|------------------------|
| Town01  | ~80개                   |
| Town02  | ~80개                   |
| Town03  | ~240개                  |
| Town04  | ~380개                  |
| Town05  | ~240개                  |
| Town06  | ~150개                  |
| Town07  | ~150개                  |
| Town10  | ~120개                  |

※ 정확한 개수는 CARLA 버전에 따라 약간 다를 수 있습니다.

## Spawn Point 구조

각 spawn point는 다음 정보를 포함합니다:

```python
spawn_point = carla.Transform(
    location=carla.Location(x, y, z),    # 위치 (미터 단위)
    rotation=carla.Rotation(pitch, yaw, roll)  # 방향 (도 단위)
)
```

### 좌표계

**CARLA 좌표계** (Unreal Engine 기반):
- X축: 전방 (Forward)
- Y축: 우측 (Right)  
- Z축: 상방 (Up)
- Left-handed 좌표계

## 이 패키지의 동작 방식

1. **Spawn Point 조회**: CARLA 맵에서 `get_spawn_points()` 함수로 모든 spawn point 리스트를 가져옵니다
2. **ID로 선택**: 사용자가 지정한 ID에 해당하는 spawn point를 선택합니다
3. **경로 계획**: 선택된 spawn point들 사이의 도로 경로를 계획합니다
4. **좌표 변환**: CARLA 좌표계를 ROS2 좌표계로 변환합니다
5. **경로 발행**: Nav2 형식으로 `/plan` 토픽에 발행합니다

## Spawn Point 확인 방법

### 방법 1: 적합한 Spawn Point 찾기 (권장!)

```bash
# 주행 차선에 있는 spawn point만 표시하고 경로 테스트
python3 scripts/find_good_spawns.py
```

출력 예시:
```
================================================================================
CARLA Spawn Point 선택 도우미
Map: Town03
================================================================================

GlobalRoutePlanner initialized ✓

Analyzing spawn points...

Total spawn points: 240
Valid driving lane spawns: 195

Recommended spawn points for path planning:

ID         X          Y    Lane   Road     Dist
--------------------------------------------------
0       -16.00      56.00  -1     10       0.50m
5      -156.00      56.00  -1     10       0.45m
10     -296.00      56.00  -1     10       0.52m
...

경로 계획 테스트:
From   To     Status       Waypoints  Note
------------------------------------------------------------
0      10     ✓ SUCCESS    125        OK
0      20     ✓ SUCCESS    248        OK
0      195    ✓ SUCCESS    512        OK
```

이 도구는:
- 주행 가능한 차선에 있는 spawn point만 표시
- Spawn point가 도로에서 얼마나 떨어져 있는지 표시
- 실제 경로 계획이 가능한지 테스트

### 방법 2: 모든 Spawn Point 보기

```bash
# 모든 spawn point 정보 (lane, road 포함)
python3 scripts/list_spawn_points.py
```

### 방법 2: 모든 Spawn Point 보기

```bash
# 모든 spawn point 정보 (lane, road 포함)
python3 scripts/list_spawn_points.py
```

출력 예시:
```
ID         X          Y          Z        Yaw    Lane     Road     ROS2_X     ROS2_Y
----------------------------------------------------------------------------------------------------
0      -16.00      56.00       0.30      90.00  -1       10       -16.00     -56.00
1     -156.00      56.00       0.30      90.00  -1       10      -156.00     -56.00
...
```

### 방법 3: CARLA에서 직접 확인

CARLA의 manual_control.py를 사용하여 시각적으로 확인:

```bash
cd $CARLA_ROOT/PythonAPI/examples
python3 manual_control.py

# 게임 화면에서:
# - 'R' 키: 다음 spawn point로 텔레포트
# - 화면에 현재 spawn point ID 표시됨
```

이 방법으로 각 spawn point의 정확한 위치를 눈으로 확인할 수 있습니다.

## 사용 예시

### 예시 1: 가까운 두 지점

```bash
# Spawn point 0에서 10으로 경로 계획
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=0 \
    goal_spawn_id:=10
```

### 예시 2: 먼 거리 경로

```bash
# Town03에서 반대편까지 (0번에서 200번으로)
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=0 \
    goal_spawn_id:=200
```

### 예시 3: 경유점 포함

```bash
# 0 → 50 → 100 → 150 순서로 경유
ros2 launch carla_path_planner path_planner.launch.py \
    start_spawn_id:=0 \
    goal_spawn_id:=150 \
    waypoint_ids:="50,100"
```

## 주의사항

1. **유효한 ID 범위**: 
   - 맵을 먼저 로드한 후 spawn point 개수를 확인하세요
   - 유효하지 않은 ID를 사용하면 에러가 발생합니다

2. **경로 계획 실패**:
   - 너무 먼 거리의 spawn point는 경로 계획이 실패할 수 있습니다
   - 도로가 연결되지 않은 spawn point 간에는 경로를 찾을 수 없습니다

3. **맵 변경**:
   - CARLA에서 맵을 변경하면 spawn point도 완전히 달라집니다
   - 맵을 변경한 후에는 spawn point를 다시 확인해야 합니다

## 맵 변경 방법

Python API로 맵 변경:

```python
import carla

client = carla.Client('localhost', 2000)
world = client.load_world('Town03')  # Town03으로 변경
```

또는 CARLA 실행 시 맵 지정:

```bash
cd CARLA_0.10.0
./CarlaUE5.sh -carla-world-port=2000 Town03
```

## 추가 팁

- **시각화**: CARLA의 spectator 모드를 사용하여 spawn point 위치를 시각적으로 확인할 수 있습니다
- **랜덤 선택**: 테스트 시 `random.choice(spawn_points)`로 무작위 spawn point를 선택할 수 있습니다
- **거리 계산**: 두 spawn point 간 직선 거리를 계산하여 적절한 경로를 선택할 수 있습니다

```python
import math

def distance(sp1, sp2):
    loc1 = sp1.location
    loc2 = sp2.location
    return math.sqrt((loc1.x - loc2.x)**2 + (loc1.y - loc2.y)**2)
```

# CARLA Path Visualizer

ROS2 Jazzy 패키지로 CARLA 시뮬레이터의 도로 정보, 경로 계획, 그리고 로봇의 실시간 위치를 2D Matplotlib 창에 시각화합니다.

## 기능

- ✅ CARLA 맵의 도로 차선 및 스폰 포인트 표시
- ✅ 계획된 경로 (Path) 시각화
- ✅ 로봇의 실시간 위치 및 방향 표시
- ✅ 로봇 이동 궤적 기록 및 표시
- ✅ 시작점과 목표점 표시
- ✅ Odometry 또는 PoseStamped 토픽 지원

## 요구사항

### 시스템
- Ubuntu 24.04
- ROS2 Jazzy
- CARLA Simulator 0.10.0 (UE5)
- Python 3.10+

### Python 패키지
```bash
pip install carla matplotlib numpy
```

### ROS2 의존성
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
cp -r carla_path_visualizer .
```

3. 빌드:
```bash
cd ~/ros2_ws
colcon build --packages-select carla_path_visualizer
source install/setup.bash
```

## 사용법

### 1. CARLA 서버 실행
먼저 CARLA 시뮬레이터를 실행하세요:
```bash
cd /path/to/carla
./CarlaUE5.sh
```

### 2. 노드 실행

#### 방법 1: 직접 실행
```bash
ros2 run carla_path_visualizer path_visualizer
```

#### 방법 2: Launch 파일 사용
```bash
ros2 launch carla_path_visualizer path_visualizer.launch.py
```

#### 방법 3: 커스텀 파라미터로 실행
```bash
ros2 launch carla_path_visualizer path_visualizer.launch.py \
  carla_host:=localhost \
  carla_port:=2000 \
  path_topic:=/plan \
  odom_topic:=/odom \
  use_odometry:=true \
  robot_size:=2.0 \
  show_trajectory:=true \
  trajectory_max_points:=1000
```

#### 방법 4: YAML 파일로 파라미터 설정
```bash
ros2 run carla_path_visualizer path_visualizer \
  --ros-args --params-file src/carla_path_visualizer/config/visualizer_params.yaml
```

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `carla_host` | string | "localhost" | CARLA 서버 호스트 주소 |
| `carla_port` | int | 2000 | CARLA 서버 포트 |
| `path_topic` | string | "/plan" | 경로 토픽 이름 |
| `odom_topic` | string | "/odom" | Odometry/Pose 토픽 이름 |
| `use_odometry` | bool | true | true: Odometry 사용, false: PoseStamped 사용 |
| `robot_size` | float | 2.0 | 로봇 마커 크기 (미터) |
| `show_trajectory` | bool | true | 로봇 궤적 표시 여부 |
| `trajectory_max_points` | int | 1000 | 저장할 최대 궤적 포인트 수 |

## 구독하는 토픽

### Path (nav_msgs/Path)
- **토픽 이름**: `/plan` (기본값)
- **설명**: 계획된 경로의 포즈 배열
- **사용**: 경로를 녹색 선으로 표시하고 시작점(파란색)과 목표점(빨간색)을 마커로 표시

### Odometry (nav_msgs/Odometry) - use_odometry=true일 때
- **토픽 이름**: `/odom` (기본값)
- **설명**: 로봇의 현재 위치와 자세
- **사용**: 로봇의 실시간 위치를 파란색 삼각형과 방향 화살표로 표시

### PoseStamped (geometry_msgs/PoseStamped) - use_odometry=false일 때
- **토픽 이름**: `/current_pose` (설정 가능)
- **설명**: 로봇의 현재 위치
- **사용**: Odometry와 동일한 방식으로 표시

## 시각화 요소

### 지도 요소
- **도로 차선**: 연한 회색 점으로 표시
- **스폰 포인트**: 빨간색 X 마커와 번호로 표시
- **좌표축**: 원점에 X축(빨강), Y축(초록) 표시

### 경로 및 로봇
- **계획된 경로**: 녹색 선
- **시작점**: 파란색 원
- **목표점**: 빨간색 원
- **로봇 위치**: 파란색 삼각형 + 방향 화살표
- **로봇 궤적**: 연한 청록색 선 (옵션)

## 좌표계 변환

CARLA와 ROS2는 다른 좌표계를 사용합니다:
- **CARLA**: 왼손 좌표계 (X: 전방, Y: 우측, Z: 상방)
- **ROS2**: 오른손 좌표계 (X: 전방, Y: 좌측, Z: 상방)

이 패키지는 자동으로 Y 좌표를 반전시켜 올바르게 표시합니다.

## 문제 해결

### CARLA 연결 실패
```bash
# CARLA 서버가 실행 중인지 확인
ps aux | grep Carla

# 포트 확인
netstat -tuln | grep 2000
```

### 토픽이 보이지 않음
```bash
# 토픽 리스트 확인
ros2 topic list

# 토픽 메시지 확인
ros2 topic echo /odom
ros2 topic echo /plan
```

### Matplotlib 창이 열리지 않음
```bash
# X11 forwarding 확인 (SSH 사용 시)
echo $DISPLAY

# Qt 백엔드 문제 시
export QT_QPA_PLATFORM=xcb
```

## 예제 사용 시나리오

### 1. 기본 사용 (Odometry 사용)
```bash
ros2 launch carla_path_visualizer path_visualizer.launch.py
```

### 2. PoseStamped 사용
```bash
ros2 launch carla_path_visualizer path_visualizer.launch.py \
  use_odometry:=false \
  odom_topic:=/current_pose
```

### 3. 궤적 표시 끄기
```bash
ros2 launch carla_path_visualizer path_visualizer.launch.py \
  show_trajectory:=false
```

### 4. 로봇 크기 조정
```bash
ros2 launch carla_path_visualizer path_visualizer.launch.py \
  robot_size:=3.0
```

## 라이센스

MIT License

## 작성자

Your Name (user@example.com)

## 버전 히스토리

- **1.0.0** (2025-02-11)
  - 초기 릴리스
  - CARLA 맵, 경로, 로봇 위치 시각화 기능
  - Odometry 및 PoseStamped 토픽 지원
  - 로봇 궤적 기록 기능

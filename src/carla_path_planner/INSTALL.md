# CARLA Path Planner 설치 가이드

## 1. 사전 요구사항 설치

### ROS2 Jazzy 설치
```bash
# Ubuntu 24.04에서 ROS2 Jazzy 설치
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Universe repository 추가
sudo apt install software-properties-common
sudo add-apt-repository universe

# ROS2 GPG 키 추가
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Repository 추가
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Jazzy Desktop 설치
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

# 개발 도구 설치
sudo apt install ros-dev-tools
```

### CARLA 0.10.0 설치
```bash
# CARLA 다운로드 (공식 웹사이트에서)
cd ~/Downloads
# https://github.com/carla-simulator/carla/releases 에서 CARLA_0.10.0.tar.gz 다운로드

# 압축 해제
mkdir -p ~/carla
tar -xzf CARLA_0.10.0.tar.gz -C ~/carla/
cd ~/carla/CARLA_0.10.0
```

### Python CARLA 패키지 설치
```bash
pip install carla==0.10.0
```

### CARLA PythonAPI 경로 설정 (필수!)

CARLA의 agents 모듈을 사용하려면 PythonAPI를 PYTHONPATH에 추가해야 합니다:

```bash
# CARLA 설치 경로 확인 및 설정
export CARLA_ROOT=$HOME/carla/CARLA_0.10.0  # 본인의 경로로 수정

# PYTHONPATH에 추가
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/agents:$PYTHONPATH
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$PYTHONPATH
```

**영구 설정을 위해 ~/.bashrc에 추가**:
```bash
echo 'export CARLA_ROOT=$HOME/carla/CARLA_0.10.0' >> ~/.bashrc
echo 'export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH' >> ~/.bashrc
echo 'export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/agents:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

### ROS2 Nav 메시지 패키지 설치
```bash
sudo apt install ros-jazzy-nav-msgs \
                 ros-jazzy-geometry-msgs \
                 ros-jazzy-std-msgs
```

## 2. 워크스페이스 생성

```bash
# ROS2 워크스페이스 생성
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 3. 패키지 복사

```bash
# 이 패키지를 워크스페이스로 복사
cp -r /path/to/carla_path_planner ~/ros2_ws/src/
```

## 4. 빌드

```bash
cd ~/ros2_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --packages-select carla_path_planner

# 환경 소싱
source install/setup.bash
```

## 5. 환경 설정 (.bashrc에 추가)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. 설치 확인

### CARLA 서버 실행
```bash
cd ~/carla/CARLA_0.10.0
./CarlaUE5.sh
```

### 패키지 확인
```bash
# 패키지 목록 확인
ros2 pkg list | grep carla_path_planner

# 노드 정보 확인
ros2 run carla_path_planner path_planner --help
```

## 7. 테스트 실행

### CARLA PythonAPI 환경 확인
```bash
# 환경 변수 확인
echo $CARLA_ROOT
echo $PYTHONPATH

# Python에서 import 테스트
python3 -c "from agents.navigation.global_route_planner import GlobalRoutePlanner; print('OK')"
```

### Spawn point 목록 확인
```bash
python3 ~/ros2_ws/src/carla_path_planner/scripts/list_spawn_points.py
```

### 경로 플래너 실행
```bash
ros2 launch carla_path_planner path_planner.launch.py
```

### 다른 터미널에서 토픽 확인
```bash
# 토픽 리스트
ros2 topic list

# /plan 토픽 데이터 확인
ros2 topic echo /plan
```

## 문제 해결

### Python 모듈 에러
```bash
# CARLA Python API가 설치되어 있는지 확인
pip show carla

# 없으면 설치
pip install carla==0.10.0
```

### CARLA 연결 에러
```bash
# CARLA 서버가 실행 중인지 확인
ps aux | grep Carla

# 포트 확인
netstat -tuln | grep 2000
```

### Colcon 빌드 에러
```bash
# 캐시 삭제 후 재빌드
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select carla_path_planner
```

## 참고 사항

- CARLA 서버는 꽤 많은 리소스를 사용합니다 (GPU 포함)
- 최소 요구사항: 8GB RAM, NVIDIA GPU (GTX 1060 이상 권장)
- UE5 맵은 처음 로딩 시 시간이 걸릴 수 있습니다

## 추가 리소스

- [ROS2 Jazzy 문서](https://docs.ros.org/en/jazzy/)
- [CARLA 문서](https://carla.readthedocs.io/)
- [Nav2 문서](https://navigation.ros.org/)

#!/bin/bash
# CARLA PythonAPI 환경 설정 스크립트

# CARLA 설치 경로 (필요시 수정)
CARLA_ROOT="${CARLA_ROOT:-$HOME/carla/CARLA_0.10.0}"

# CARLA PythonAPI 경로 확인
if [ ! -d "$CARLA_ROOT/PythonAPI" ]; then
    echo "Error: CARLA PythonAPI not found at $CARLA_ROOT/PythonAPI"
    echo "Please set CARLA_ROOT environment variable to your CARLA installation directory"
    echo "Example: export CARLA_ROOT=/path/to/CARLA_0.10.0"
    exit 1
fi

# PythonAPI를 PYTHONPATH에 추가
export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH"

# carla dist 패키지 경로도 추가 (egg 파일)
CARLA_EGG=$(find "$CARLA_ROOT/PythonAPI/carla/dist" -name "carla-*.egg" 2>/dev/null | head -1)
if [ -n "$CARLA_EGG" ]; then
    export PYTHONPATH="$CARLA_EGG:$PYTHONPATH"
    echo "Added CARLA egg: $CARLA_EGG"
fi

# agents 모듈 경로 추가
export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla/agents:$PYTHONPATH"
export PYTHONPATH="$CARLA_ROOT/PythonAPI:$PYTHONPATH"

echo "CARLA PythonAPI paths added to PYTHONPATH"
echo "CARLA_ROOT: $CARLA_ROOT"
echo ""
echo "You can now run:"
echo "  ros2 launch carla_path_planner path_planner.launch.py"
echo ""
echo "To make this permanent, add to your ~/.bashrc:"
echo "  export CARLA_ROOT=$CARLA_ROOT"
echo "  source $(pwd)/setup_carla_env.sh"

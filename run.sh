#!/bin/bash
set -e

# 스크립트가 위치한 폴더 경로 추출 (src/robot_vision)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 워크스페이스 루트 폴더로 이동 (robot_vision 폴더의 상위 상위 폴더인 workspace 루트로 이동)
cd "$SCRIPT_DIR/../.."

echo "🚀 [1/3] robot_vision 패키지 빌드를 시작합니다..."
colcon build --packages-select robot_vision

echo "⚙️ [2/3] ROS 2 환경 변수를 적용합니다..."
source install/setup.bash

echo "🤖 [3/3] 통합 Launch 파일을 실행합니다 (Gazebo + Bridge + YOLO)..."
ros2 launch robot_vision vision_bringup.launch.py

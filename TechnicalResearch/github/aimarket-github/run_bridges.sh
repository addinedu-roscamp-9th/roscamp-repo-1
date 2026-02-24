#!/bin/bash
# 서버 사이드 ROS 2 브릿지 실행 스크립트
# Arm 17 브릿지 (Domain 7)를 백그라운드로 시작합니다.
#
# [환경변수]
#   ARM17_IP   : 로봇팔 17번 IP
#   ARM17_USER : SSH 사용자명
#   ARM17_PASS : SSH 비밀번호
#
# [사용법]
#   chmod +x run_bridges.sh
#   ARM17_IP=192.168.0.17 ARM17_USER=jetcobot ARM17_PASS=yourpw ./run_bridges.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_SCRIPT="${SCRIPT_DIR}/robot/common/arm_server_ros2.py"
LOG_FILE="${SCRIPT_DIR}/arm17_bridge.log"

echo "Starting Arm 17 ROS 2 Bridge (Domain 7)..."
pkill -f "arm_server_ros2.py" 2>/dev/null || true

export ROS_DOMAIN_ID=7
nohup python3 "${BRIDGE_SCRIPT}" 17 > "${LOG_FILE}" 2>&1 &

echo "✅ Arm 17 Bridge Running (Log: ${LOG_FILE})"

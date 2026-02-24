#!/bin/bash
# 로봇팔 18번 트리거 스크립트
# 환경변수 설정 필요:
#   ARM18_IP    : 로봇팔 18번 IP 주소 (예: 192.168.0.xxx)
#   ARM18_USER  : SSH 사용자 이름 (예: jetcobot)
#   ARM18_PASS  : SSH 비밀번호
set -e

ARM18_IP="${ARM18_IP:-192.168.0.xxx}"
ARM18_USER="${ARM18_USER:-jetcobot}"
ARM18_PASS="${ARM18_PASS:-your_password}"

echo "로봇팔 18 (mycobot 환경) 실행 시도 중..."
sshpass -p "$ARM18_PASS" ssh ${ARM18_USER}@${ARM18_IP} \
    "source /home/${ARM18_USER}/venv/mycobot/bin/activate && cd /home/${ARM18_USER}/aruco_pick && python3 check.py"

echo "로봇팔 18 작업 완료"

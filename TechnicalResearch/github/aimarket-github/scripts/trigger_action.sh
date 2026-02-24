#!/bin/bash
# Trigger Remote Action on Pinky Robot via SSH
# 환경변수 설정 필요:
#   PINKY_IP    : Pinky 로봇 IP 주소
#   PINKY_USER  : SSH 사용자 이름
#   PINKY_PASS  : SSH 비밀번호

ACTION_JSON=$1

if [ -z "$ACTION_JSON" ]; then
    echo "Usage: $0 '<json_string>'"
    exit 1
fi

PINKY_IP="${PINKY_IP:-192.168.0.xxx}"
PINKY_USER="${PINKY_USER:-pinky}"
PINKY_PASS="${PINKY_PASS:-your_password}"

echo "Triggering action: $ACTION_JSON"

# SSH and run the action runner on the robot
sshpass -p "$PINKY_PASS" ssh -o StrictHostKeyChecking=no ${PINKY_USER}@${PINKY_IP} \
"export ROS_DOMAIN_ID=0 && \
 source /opt/ros/jazzy/setup.bash && \
 source /home/${PINKY_USER}/pinky_pro/install/local_setup.bash && \
 python3 /home/${PINKY_USER}/action_runner.py '$ACTION_JSON'"

if [ $? -eq 0 ]; then
    echo "✅ Action triggered successfully."
else
    echo "⚠️ Action trigger failed."
    exit 1
fi

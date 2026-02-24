#!/bin/bash
# Autonomous Robot Pinky Phased Trigger (123=Arrival, lcd=Delivery)
# 환경변수 설정 필요:
#   PINKY_IP      : Pinky 로봇의 IP 주소 (예: 192.168.0.xxx)
#   PINKY_USER    : SSH 사용자 이름 (예: pinky)
#   PINKY_PASS    : SSH 비밀번호

MISSION=$1
PINKY_IP="${PINKY_IP:-192.168.0.xxx}"
PINKY_USER="${PINKY_USER:-pinky}"
PINKY_PASS="${PINKY_PASS:-your_password}"

if [ "$MISSION" == "123" ]; then
    echo "Executing Pinky Arrival Mission (123) - Blocking..."
    sshpass -p "$PINKY_PASS" ssh -o StrictHostKeyChecking=no ${PINKY_USER}@${PINKY_IP} \
    "export ROS_DOMAIN_ID=19 && \
     source /opt/ros/jazzy/setup.bash && \
     source /home/${PINKY_USER}/pinky_pro/install/local_setup.bash && \
     python3 /home/${PINKY_USER}/waypoint_123.py"
elif [ "$MISSION" == "lcd" ]; then
    echo "Executing Pinky Delivery Mission (lcd) - Background..."
    sshpass -p "$PINKY_PASS" ssh -o StrictHostKeyChecking=no ${PINKY_USER}@${PINKY_IP} \
    "nohup bash -c 'export ROS_DOMAIN_ID=19 && \
     source /opt/ros/jazzy/setup.bash && \
     source /home/${PINKY_USER}/pinky_pro/install/local_setup.bash && \
     python3 /home/${PINKY_USER}/waypoint_lcd.py' > /home/${PINKY_USER}/mission.log 2>&1 &"
else
    echo "Usage: $0 [123|lcd]"
    exit 1
fi

if [ $? -eq 0 ]; then
    echo "✅ Pinky mission ($MISSION) handled successfully."
else
    echo "⚠️ Failed to handle Pinky mission ($MISSION)."
    exit 1
fi

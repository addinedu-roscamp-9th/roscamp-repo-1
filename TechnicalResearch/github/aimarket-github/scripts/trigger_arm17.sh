#!/bin/bash
# 로봇팔 17번 트리거 스크립트 (인자에 따라 다른 파일 실행)
# 환경변수 설정 필요:
#   ARM17_IP    : 로봇팔 17번 IP 주소 (예: 192.168.0.xxx)
#   ARM17_USER  : SSH 사용자 이름 (예: jetcobot)
#   ARM17_PASS  : SSH 비밀번호

MARKER=$1
ARM17_IP="${ARM17_IP:-192.168.0.xxx}"
ARM17_USER="${ARM17_USER:-jetcobot}"
ARM17_PASS="${ARM17_PASS:-your_password}"

if [ "$MARKER" == "6" ]; then
    echo "실행: 6번 마커 전용 코드 (step1_aruco_check6.py)"
    sshpass -p "$ARM17_PASS" ssh ${ARM17_USER}@${ARM17_IP} \
        "/home/${ARM17_USER}/photo_upload_bundle/client/.venv/bin/python /home/${ARM17_USER}/aruco_pick/step1_aruco_check6.py"
elif [ "$MARKER" == "37" ]; then
    echo "실행: 37번 마커 전용 코드 (step1_aruco_check37.py)"
    sshpass -p "$ARM17_PASS" ssh ${ARM17_USER}@${ARM17_IP} \
        "/home/${ARM17_USER}/photo_upload_bundle/client/.venv/bin/python /home/${ARM17_USER}/aruco_pick/step1_aruco_check37.py"
else
    echo "실행: 전체 마커 확인 코드 (step1_aruco_check.py)"
    sshpass -p "$ARM17_PASS" ssh ${ARM17_USER}@${ARM17_IP} \
        "source /home/${ARM17_USER}/mycobot_venv/bin/activate && python3 /home/${ARM17_USER}/aruco_pick/step1_aruco_check.py"
fi

echo "로봇팔 17 작업 완료 (인자: $MARKER)"

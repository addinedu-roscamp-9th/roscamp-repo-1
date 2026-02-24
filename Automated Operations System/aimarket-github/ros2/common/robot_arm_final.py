#!/usr/bin/env python3
"""
Robot Arm Flask Server - Arm 17 / Arm 18 (로봇 사이드 실행 파일)
SSH로 연결된 후 실행되는 Flask HTTP 서버 + ArUco 마커 기반 픽앤플레이스

[통신 방식]
  FastAPI 서버 → SSH → 이 스크립트 실행 (trigger_arm17.sh / trigger_arm18.sh)
  또는 ROS 2 토픽 수신 후 arm_server_ros2.py가 SSH로 이 스크립트를 호출

[REST API 엔드포인트 (로봇 내부 Flask)]
  GET  /health        - 상태 확인
  POST /pick_items    - 픽앤플레이스 명령 수신
    Body: { "order_id": 1, "marker_ids": [6, 37] }

[환경변수 / 수정 필요 항목]
  ROBOT_NAME      : 로봇 이름 (Arm-1 또는 Arm-2)
  SERVER_API_URL  : FastAPI 서버 URL (예: http://192.168.0.xxx:8000/api/robot)
  SERIAL_PORT     : 로봇 시리얼 포트 (기본값: /dev/ttyJETCOBOT)
"""
import time
import threading
import requests
import json
import os
from queue import Queue, Empty

from flask import Flask, request, jsonify
import cv2
import numpy as np
from pymycobot.mycobot import MyCobot

app = Flask(__name__)

# ============================================================
# 0) 로봇별 설정 (환경변수 또는 직접 수정)
# ============================================================
ROBOT_NAME    = os.environ.get("ROBOT_NAME", "Arm-1")
SERIAL_PORT   = os.environ.get("SERIAL_PORT", "/dev/ttyJETCOBOT")
SERIAL_BAUD   = 1000000
SERVER_API_URL = os.environ.get("SERVER_API_URL", "http://192.168.0.xxx:8000/api/robot")
MARKER_LENGTH_M = 0.03

# 픽 좌표 (ArUco 마커 ID별로 설정)
PICK_COORDS = {
    6:  [240.6, -40.0, 128.9, -174.54,  8.55,  50.59],
    7:  [240.6, -40.0, 128.9, -174.54,  8.55,  50.59],
    37: [240.6, -40.0, 128.9, -174.54,  8.55,  50.59],
}

START_COORDS    = [150.2, -31.3, 255.0, -177.0, -3.07, -130.3]
PRE_DROP_COORDS = [33.2, -0.3, 394.4, 99.85, -10.36, -144.25]
DROP_COORDS     = [-63.9, 241.9, 175.7, 159.52, 6.06, -118.74]

SPEED_START = 15; SPEED_APPROACH = 12; SPEED_PICK = 8
SPEED_LIFT = 12;  SPEED_PRE_DROP = 12; SPEED_DROP = 12; SPEED_DROP_DN = 8
APPROACH_UP_MM = 70.0; LIFT_UP_MM = 90.0; DROP_UP_MM = 70.0
Z_MIN = -70.0; Z_MAX = 412.67
STABLE_FRAMES = 8; TIMEOUT_S = 5.0
DRY_RUN = False

# ============================================================
# 1) ArUco 검출기
# ============================================================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector   = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

# ============================================================
# 2) 로봇 초기화
# ============================================================
try:
    mc = MyCobot(SERIAL_PORT, SERIAL_BAUD)
    mc.thread_lock = True
except Exception as e:
    print(f"[{ROBOT_NAME}] 로봇 연결 실패 (Demo Mode?): {e}")
    mc = None

def gripper_open():
    if mc and not DRY_RUN:
        mc.set_gripper_state(0, 80); time.sleep(0.3)

def gripper_close():
    if mc and not DRY_RUN:
        mc.set_gripper_state(1, 80); time.sleep(0.3)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def move_coords(coords, speed, wait):
    if mc is None: return
    c = list(coords); c[2] = clamp(c[2], Z_MIN, Z_MAX)
    if DRY_RUN:
        print(f"[{ROBOT_NAME}] (DRY_RUN) -> {c}"); time.sleep(0.05); return
    mc.send_coords(c, speed, 0); time.sleep(wait)

def go_start():
    move_coords(START_COORDS, SPEED_START, 3.5)

# ============================================================
# 3) 마커 확인
# ============================================================
def wait_for_marker(target_id):
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    stable = 0; t0 = time.time()
    try:
        while True:
            if time.time() - t0 > TIMEOUT_S: return False
            ret, frame = cap.read()
            if not ret: continue
            corners, ids, _ = detector.detectMarkers(frame)
            found = ids is not None and any(int(i[0]) == int(target_id) for i in ids)
            stable = stable + 1 if found else 0
            if stable >= STABLE_FRAMES: return True
    finally:
        cap.release()

# ============================================================
# 4) 픽앤플레이스
# ============================================================
def pick_and_place(mid):
    if int(mid) not in PICK_COORDS: return False
    px, py, pz, prx, pry, prz = PICK_COORDS[int(mid)]
    prex, prey, prez, prerx, prery, prerz = PRE_DROP_COORDS
    dx, dy, dz, drx, dry, drz = DROP_COORDS

    gripper_open()
    move_coords([px, py, pz+APPROACH_UP_MM, prx, pry, prz], SPEED_APPROACH, 3.0)
    move_coords([px, py, pz, prx, pry, prz], SPEED_PICK, 3.0)
    gripper_close(); time.sleep(0.5)
    move_coords([px, py, pz+LIFT_UP_MM, prx, pry, prz], SPEED_LIFT, 3.0)
    move_coords([prex, prey, prez, prerx, prery, prerz], SPEED_PRE_DROP, 3.0)
    move_coords([dx, dy, dz+DROP_UP_MM, drx, dry, drz], SPEED_DROP, 3.0)
    move_coords([dx, dy, dz, drx, dry, drz], SPEED_DROP_DN, 3.0)
    gripper_open(); time.sleep(0.5)
    move_coords([dx, dy, dz+DROP_UP_MM, drx, dry, drz], SPEED_DROP, 3.0)
    return True

# ============================================================
# 5) 서버 피드백
# ============================================================
def notify_server(endpoint, data):
    url = f"{SERVER_API_URL}/{endpoint}"
    try:
        data["robot_name"] = ROBOT_NAME
        requests.post(url, json=data, timeout=1.0)
    except Exception as e:
        print(f"⚠️ 서버 알림 실패 ({endpoint}): {e}")

# ============================================================
# 6) 작업 큐 + 워커
# ============================================================
job_q: "Queue[dict]" = Queue()

def worker_loop():
    print(f"[{ROBOT_NAME}] ✅ 워커 시작")
    if mc: gripper_open(); go_start()
    while True:
        try:
            job = job_q.get(timeout=0.5)
        except Empty:
            continue
        order_id = job.get("order_id"); markers = job.get("marker_ids", [])
        print(f"[{ROBOT_NAME}] ▶️ Order {order_id}: {markers}")
        try:
            for mid in markers:
                mid = int(mid)
                notify_server("task/start", {"order_id": order_id, "marker_id": mid})
                if wait_for_marker(mid):
                    ok = pick_and_place(mid)
                    endpoint = "task/success" if ok else "task/fail"
                    notify_server(endpoint, {"order_id": order_id, "marker_id": mid})
                else:
                    notify_server("task/fail", {"order_id": order_id, "marker_id": mid, "reason": "timeout"})
            if mc: go_start()
        except Exception as e:
            notify_server("task/fail", {"order_id": order_id, "marker_id": -1, "reason": str(e)})
        finally:
            job_q.task_done()

# ============================================================
# 7) Flask 앤드포인트
# ============================================================
@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "robot": ROBOT_NAME, "queued": job_q.qsize()}), 200

@app.route("/pick_items", methods=["POST"])
def pick_items():
    data = request.get_json(force=True, silent=True) or {}
    order_id = data.get("order_id"); markers = data.get("marker_ids", [])
    if not isinstance(markers, list):
        return jsonify({"status": "error", "reason": "marker_ids must be a list"}), 400
    print(f"[{ROBOT_NAME}] 📦 Order {order_id}: markers={markers}")
    job_q.put({"order_id": order_id, "marker_ids": markers})
    notify_server("ack", {"order_id": order_id, "status": "accepted"})
    return jsonify({"status": "accepted", "robot": ROBOT_NAME, "queued": job_q.qsize()}), 200

def main():
    threading.Thread(target=worker_loop, daemon=True).start()
    app.run(host="0.0.0.0", port=28080)

if __name__ == "__main__":
    main()

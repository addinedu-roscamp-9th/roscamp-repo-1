#!/usr/bin/env python3
import time
import cv2
import numpy as np
from pymycobot.mycobot import MyCobot

# ============================================================
# 0) 설정
# ============================================================
MARKER_LENGTH_M = 0.03

# ✅ 마커 ID
TARGET_MARKER_ID = 7

# ✅ 고정 픽 좌표
PICK_COORDS = {
    7: [240.6, -40.0, 128.9, -174.54, 8.55, 50.59],
}

# ✅ 시작 위치
START_COORDS = [150.2, -31.3, 255.0, -177.0, -3.07, -130.3]

# ✅ (추가) 내려놓기 전에 경유할 좌표
PRE_DROP_COORDS = [33.2, -0.3, 394.4, 99.85, -10.36, -144.25]

# ✅ 적재 위치
DROP_COORDS = [-63.9, 241.9, 175.7, 159.52, 6.06, -118.74]

# 속도
SPEED_START = 15
SPEED_APPROACH = 12
SPEED_PICK = 8
SPEED_LIFT = 12
SPEED_PRE_DROP = 12  # ✅ 경유용 속도(추가)
SPEED_DROP = 12
SPEED_DROP_DN = 8

# 동작 파라미터
APPROACH_UP_MM = 70.0
LIFT_UP_MM = 90.0
DROP_UP_MM = 70.0

Z_MIN = -70.0
Z_MAX = 412.67

# 마커 인식 조건
STABLE_FRAMES = 8
TIMEOUT_S = 5.0

# ============================================================
# 1) 카메라 (검출용)
# ============================================================
CAMERA_MATRIX = np.array(
    [
        [861.099310176272, 0.0, 397.2018875648457],
        [0.0, 865.1910968062635, 190.88958278817674],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

DIST_COEFFS = np.array(
    [
        -0.46538265851387517,
        0.4358867472140529,
        0.007540959571111467,
        -0.008265561645724185,
        1.8996374779477747,
    ],
    dtype=np.float64,
)

aruco = cv2.aruco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

# ============================================================
# 2) 로봇
# ============================================================
mc = MyCobot("/dev/ttyJETCOBOT", 1000000)
mc.thread_lock = True


def gripper_open():
    mc.set_gripper_state(0, 80)
    time.sleep(0.3)


def gripper_close():
    mc.set_gripper_state(1, 80)
    time.sleep(0.3)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def move_coords(coords, speed, wait):
    c = list(coords)
    c[2] = clamp(c[2], Z_MIN, Z_MAX)
    mc.send_coords(c, speed, 0)
    time.sleep(wait)


def go_start():
    move_coords(START_COORDS, SPEED_START, 3.5)


# ============================================================
# 3) 마커 확인
# ============================================================
def wait_for_marker(target_id: int) -> bool:
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    stable = 0
    t0 = time.time()

    try:
        while True:
            if time.time() - t0 > TIMEOUT_S:
                return False

            ret, frame = cap.read()
            if not ret:
                continue

            _, ids, _ = detector.detectMarkers(frame)

            found = False
            if ids is not None:
                for i in range(len(ids)):
                    if int(ids[i][0]) == target_id:
                        found = True
                        break

            stable = stable + 1 if found else 0

            if stable >= STABLE_FRAMES:
                return True
    finally:
        cap.release()
        cv2.destroyAllWindows()


# ============================================================
# 4) 픽앤플레이스
# ============================================================
def pick_and_place(mid: int):
    px, py, pz, prx, pry, prz = PICK_COORDS[mid]
    prex, prey, prez, prerx, prery, prerz = PRE_DROP_COORDS
    dx, dy, dz, drx, dry, drz = DROP_COORDS

    gripper_open()

    # 접근(위에서)
    move_coords([px, py, pz + APPROACH_UP_MM, prx, pry, prz], SPEED_APPROACH, 3.0)
    # 내려가기
    move_coords([px, py, pz, prx, pry, prz], SPEED_PICK, 3.0)

    # 집기
    gripper_close()
    time.sleep(0.5)

    # 들어올리기
    move_coords([px, py, pz + LIFT_UP_MM, prx, pry, prz], SPEED_LIFT, 3.0)

    # ✅ 적재 전에 경유 좌표로 먼저 이동
    move_coords([prex, prey, prez, prerx, prery, prerz], SPEED_PRE_DROP, 3.0)

    # 적재 위치로 이동 + 내려놓기
    move_coords([dx, dy, dz + DROP_UP_MM, drx, dry, drz], SPEED_DROP, 3.0)
    move_coords([dx, dy, dz, drx, dry, drz], SPEED_DROP_DN, 3.0)

    gripper_open()
    time.sleep(0.5)

    # 다시 위로
    move_coords([dx, dy, dz + DROP_UP_MM, drx, dry, drz], SPEED_DROP, 3.0)


# ============================================================
# 5) 메인 로직
# ============================================================
def main():
    print("START → (ID=7 보이면 집기) → PRE_DROP → DROP → START → 종료")

    gripper_open()
    go_start()

    if wait_for_marker(TARGET_MARKER_ID):
        print(f"ID={TARGET_MARKER_ID} 발견 → 집기")
        pick_and_place(TARGET_MARKER_ID)
    else:
        print(f"ID={TARGET_MARKER_ID} 없음")

    go_start()
    print("종료")


if __name__ == "__main__":
    main()

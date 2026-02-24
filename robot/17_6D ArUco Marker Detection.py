#!/usr/bin/env python3
import os
import cv2
import numpy as np
import math
import time

# ====== 너 캘리브레이션 값(12d0) ======
CAMERA_MATRIX = np.array([
    [861.099310176272, 0.0, 397.2018875648457],
    [0.0, 865.1910968062635, 190.88958278817674],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

DIST_COEFFS = np.array([
    -0.46538265851387517,
    0.4358867472140529,
    0.007540959571111467,
    -0.008265561645724185,
    1.8996374779477747
], dtype=np.float64)

MARKER_LENGTH_M = 0.03  # 3cm

# 창 띄울 수 있으면 띄우고, DISPLAY 없으면 터미널 출력만
SHOW_WINDOW = bool(os.environ.get("DISPLAY"))

aruco = cv2.aruco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 파라미터 생성(버전 호환)
try:
    params = aruco.DetectorParameters_create()
except AttributeError:
    params = aruco.DetectorParameters()

# 신형 OpenCV 방식
if not hasattr(aruco, "ArucoDetector"):
    raise SystemExit("❌ ArucoDetector 없음: OpenCV 버전/설치 확인 필요")
detector = aruco.ArucoDetector(aruco_dict, params)

# 포즈 추정 함수 확인(보통 opencv-contrib 필요)
if not hasattr(aruco, "estimatePoseSingleMarkers"):
    raise SystemExit("❌ estimatePoseSingleMarkers 없음: opencv-contrib-python 필요")

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    raise SystemExit("❌ 카메라 열기 실패")

def rvec_to_euler_deg(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = 0.0
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

print("✅ ArUco 6D+Dist 출력 시작")
print(f"- SHOW_WINDOW={SHOW_WINDOW} (DISPLAY={os.environ.get('DISPLAY')})")
print("- 종료: 창 있으면 ESC / 없으면 Ctrl+C")

last_print = 0.0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임 읽기 실패")
            continue

        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None and len(ids) > 0:
            if SHOW_WINDOW:
                aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, MARKER_LENGTH_M, CAMERA_MATRIX, DIST_COEFFS
            )

            now = time.time()
            for i in range(len(ids)):
                mid = int(ids[i][0])
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]  # meters

                X, Y, Z = map(float, tvec)
                dist = math.sqrt(X*X + Y*Y + Z*Z)
                roll, pitch, yaw = rvec_to_euler_deg(rvec)

                # 터미널 출력 (0.3초에 한 번씩)
                if now - last_print > 0.3:
                    print(f"[ID {mid}] X={X:.3f} Y={Y:.3f} Z={Z:.3f} m | "
                          f"R={roll:.1f} P={pitch:.1f} Y={yaw:.1f} deg | Dist={dist:.3f} m")

                # 화면 텍스트 표시
                if SHOW_WINDOW:
                    # 마커 중심 근처에 텍스트 붙이기
                    cx = int(corners[i][0][:, 0].mean())
                    cy = int(corners[i][0][:, 1].mean())
                    lines = [
                        f"ID:{mid}",
                        f"X={X:.3f} Y={Y:.3f} Z={Z:.3f} m",
                        f"R={roll:.1f} P={pitch:.1f} Y={yaw:.1f} deg",
                        f"Dist={dist:.3f} m",
                    ]
                    for j, t in enumerate(lines):
                        cv2.putText(frame, t, (cx + 10, cy - 55 + j*18),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)

                    # 축 그리기(경고 뜨면 무시 가능)
                    try:
                        cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, MARKER_LENGTH_M * 0.5)
                    except Exception:
                        pass

            if now - last_print > 0.3:
                last_print = now

        else:
            if SHOW_WINDOW:
                cv2.putText(frame, "No marker", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

        if SHOW_WINDOW:
            cv2.imshow("ArUco 6D + Distance", frame)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break

finally:
    cap.release()
    if SHOW_WINDOW:
        cv2.destroyAllWindows()

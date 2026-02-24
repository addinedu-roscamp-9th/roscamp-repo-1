import cv2
import numpy as np
import math

# =========================
# 카메라 캘리브레이션 (1645)
# =========================
camera_matrix = np.array([
    [992.2572499557648, 0.0, 366.2474713098142],
    [0.0, 991.1925603090785, 237.95156270390345],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

dist_coeffs = np.array([
    -0.4497357424333291,
    1.1884522348712647,
    0.005677915195943356,
    0.0009411844915971025,
    -6.384888266222218
], dtype=np.float64)

# =========================
# ArUco 설정 (4x4)
# =========================
aruco = cv2.aruco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
params = aruco.DetectorParameters()
params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

detector = aruco.ArucoDetector(aruco_dict, params)

# =========================
# 마커 크기 (3cm)
# =========================
MARKER_SIZE_CM = 3.0
half = MARKER_SIZE_CM / 2.0

obj_pts = np.array([
    [-half,  half, 0.0],
    [ half,  half, 0.0],
    [ half, -half, 0.0],
    [-half, -half, 0.0],
], dtype=np.float64)

# =========================
# 회전 → RPY
# =========================
def rvec_to_rpy_deg(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    roll  = math.atan2(R[2,1], R[2,2])
    pitch = math.atan2(-R[2,0], sy)
    yaw   = math.atan2(R[1,0], R[0,0])
    return map(math.degrees, (roll, pitch, yaw))

# =========================
# 카메라
# =========================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("❌ 카메라 열기 실패")
    exit()

print("✅ 6D 출력 (단위 cm / deg)")
print("✅ 거리 = Z값만 사용")
print("ESC 종료")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            img_pts = corners[i].reshape(4, 2)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not ok:
                continue

            x, y, z = tvec.flatten()   # cm
            roll, pitch, yaw = rvec_to_rpy_deg(rvec)

            cx = int(img_pts[:,0].mean())
            cy = int(img_pts[:,1].mean())

            cv2.putText(frame,
                f"pos(cm)=({x:.1f},{y:.1f},{z:.1f})",
                (cx+10, cy-30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            cv2.putText(frame,
                f"rpy(deg)=({roll:.1f},{pitch:.1f},{yaw:.1f})",
                (cx+10, cy-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            cv2.putText(frame,
                f"distance(Z)={z:.1f}cm",
                (cx+10, cy+10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("aruco 6D correct", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()

import numpy as np
import cv2
import os
import argparse


def calibrate(dirpath, square_size, width, height, visualize=False):
    """Apply camera calibration operation for images in the given directory path."""

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((height * width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    objp = objp * square_size

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    images = os.listdir(dirpath)

    for fname in images:
        img_path = os.path.join(dirpath, fname)
        img = cv2.imread(img_path)

        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            imgpoints.append(corners2)

            img = cv2.drawChessboardCorners(
                img, (width, height), corners2, ret
            )

        if visualize:
            cv2.imshow("img", img)
            cv2.waitKey(0)

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    return ret, mtx, dist, rvecs, tvecs


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-d",
        "--dir",
        required=True,
        help="Path to folder containing checkerboard images"
    )
    ap.add_argument(
        "-w",
        "--width",
        type=int,
        default=8,
        help="Checkerboard width (default=8)"
    )
    ap.add_argument(
        "-t",
        "--height",
        type=int,
        default=6,
        help="Checkerboard height (default=6)"
    )
    ap.add_argument(
        "-s",
        "--square_size",
        type=float,
        default=1,
        help="Size of one square (in meters)"
    )
    ap.add_argument(
        "-v",
        "--visualize",
        type=str,
        default="False",
        help="Visualize detection (True/False)"
    )

    args = vars(ap.parse_args())

    dirpath = args["dir"]
    square_size = args["square_size"]
    width = args["width"]
    height = args["height"]
    visualize = args["visualize"].lower() == "true"

    ret, mtx, dist, rvecs, tvecs = calibrate(
        dirpath,
        square_size,
        width,
        height,
        visualize=visualize
    )

    print(f"Camera Matrix:\n{mtx}")
    print(f"Distortion Coefficients:\n{dist}")

    np.save("calibration_matrix.npy", mtx)
    np.save("distortion_coefficients.npy", dist)

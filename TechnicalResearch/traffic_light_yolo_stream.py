#!/usr/bin/env python3
import time
import threading

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from ultralytics import YOLO
from picamera2 import Picamera2

from flask import Flask, Response

try:
    from pinkylib import LED as PinkyLED
    PINKY_LED_OK = True
except Exception:
    PINKY_LED_OK = False


MODEL_PATH = "yolov8n.pt"
TARGET_CLASS_NAME = "traffic light"
CONF_TH = 0.25

DETECT_HZ = 8.0

YOLO_INFER_W, YOLO_INFER_H = 256, 192

MUX_PUB_HZ = 200

ROI_CENTER_RATIO = 0.60

RED_S_MIN, RED_V_MIN = 20, 35
GREEN_S_MIN, GREEN_V_MIN = 60, 70

RED_H1 = (0, 25)
RED_H2 = (155, 179)
GREEN_H = (50, 75)

COLOR_SCORE_TH = 0.03
RATIO_TH = 1.2

RED_CONSEC = 1
GREEN_CONSEC = 3

DEBUG_PRINT = False

CAM_W, CAM_H = 640, 480
CAMERA_OUTPUT_IS_RGB = False

ROTATE_180 = True

LED_ENABLE = True
LED_BRIGHTNESS = 150

WEB_ENABLE = True
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000

STREAM_FPS = 18
STREAM_JPEG_QUALITY = 50
STREAM_MAX_W = 480

CAPTURE_FPS = 30


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def center_crop_bbox(x1, y1, x2, y2, ratio):
    w = x2 - x1
    h = y2 - y1
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    nw = w * ratio
    nh = h * ratio
    nx1 = int(cx - nw / 2.0)
    ny1 = int(cy - nh / 2.0)
    nx2 = int(cx + nw / 2.0)
    ny2 = int(cy + nh / 2.0)
    return nx1, ny1, nx2, ny2


def hsv_color_scores(bgr_roi):
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    H = hsv[:, :, 0].astype(np.int16)
    S = hsv[:, :, 1].astype(np.float32)
    V = hsv[:, :, 2].astype(np.float32)

    red_mask = (
        ((H >= RED_H1[0]) & (H <= RED_H1[1])) |
        ((H >= RED_H2[0]) & (H <= RED_H2[1]))
    )
    green_mask = (H >= GREEN_H[0]) & (H <= GREEN_H[1])

    red_mask = red_mask & (S >= RED_S_MIN) & (V >= RED_V_MIN)
    green_mask = green_mask & (S >= GREEN_S_MIN) & (V >= GREEN_V_MIN)

    score_map = (S / 255.0) * (V / 255.0)
    red_score = float(np.mean(score_map[red_mask])) if np.any(red_mask) else 0.0
    green_score = float(np.mean(score_map[green_mask])) if np.any(green_mask) else 0.0

    s_mean = float(np.mean(S))
    v_mean = float(np.mean(V))
    return red_score, green_score, s_mean, v_mean


def resize_max_w(img, max_w):
    h, w = img.shape[:2]
    if w <= max_w:
        return img
    scale = max_w / float(w)
    return cv2.resize(img, (int(w * scale), int(h * scale)))


app = Flask(__name__)
_latest_jpeg = None
_latest_jpeg_lock = threading.Lock()


@app.route("/")
def index():
    return (
        "<h2>Pinky YOLO Preview (Smooth)</h2>"
        "<p><a href='/video'>/video</a></p>"
        "<img src='/video' />"
    )


def mjpeg_generator():
    global _latest_jpeg
    last = None
    period = 1.0 / float(STREAM_FPS)
    while True:
        with _latest_jpeg_lock:
            frame = _latest_jpeg

        if frame is None or frame == last:
            time.sleep(period)
            continue

        last = frame
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(period)


@app.route("/video")
def video_feed():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


def start_web_server():
    app.run(host=WEB_HOST, port=WEB_PORT, threaded=True)


# 이후 클래스 부분은 기존 코드와 동일 (한국어 주석만 제거된 상태)

from flask import Flask, Response
from pinkylib import Camera
import cv2
from ultralytics import YOLO

app = Flask(__name__)

# YOLO 모델 (가벼운 yolov8n 사용 추천)
model = YOLO('yolov8n.pt')  # 처음 실행 시 자동 다운로드

# Pinky 카메라 초기화
cam = Camera()
cam.start(width=640, height=480)


def generate_frames():
    while True:
        frame = cam.get_frame()
        if frame is None:
            continue

        # YOLO 추론
        results = model.predict(source=frame, conf=0.5, verbose=False)

        annotated = frame.copy()

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]

                # 신호등(traffic light)만 표시
                if cls_name != 'traffic light':
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                # 박스 그리기
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{cls_name} {conf:.2f}'
                cv2.putText(
                    annotated,
                    label,
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

        # JPEG 인코딩
        ret, buffer = cv2.imencode('.jpg', annotated)
        if not ret:
            continue

        jpg_bytes = buffer.tobytes()

        # MJPEG 스트림 포맷으로 전송
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + jpg_bytes + b'\r\n'
        )


@app.route('/video')
def video():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        cam.close()

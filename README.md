ROS2와 AI를 활용한 **AI 기반 지능형 무인 마트 자동화 시스템**  
(결제 이후: 작업 생성 → 로봇팔 집기/적재 → 자율주행 운반 → 비전 기반 주행 제어)

**�� Presentation:** (링크)  |  **�� Video Demo:** (링크)

---

> 본 프로젝트는 사용자의 **결제 완료 시점부터 상품 전달까지**의 물류 과정을  
> **로봇팔 + 자율주행 로봇 + AI 비전**으로 통합 자동화한 시스템입니다.

## 프로젝트 개요

- **프로젝트명:** AI 기반 지능형 무인 마트 자동화 시스템
- **주제:** 결제 이후 상품 배송 전 과정 로봇 자동화
- **핵심 목표:** 결제 이후 프로세스를 하나의 자동화 파이프라인으로 연결  
  (서버 작업 생성 → 로봇팔 피킹/적재 → Pinky 자율주행 운반 → 신호등 인식 정지)

---

## MAP
<img width="600" height="1260" alt="제목을 입력해주세요" src="https://github.com/user-attachments/assets/35a33084-b374-4e5b-a7ca-e3fea746ff4e" />

---
## 팀 구성 및 역할

| Name | Role |
|------|------|
| (이준호) | Manipulator(로봇팔), ArUco 인식/집기, 서버 연동 |
| (이현서) | Manipulator(로봇팔), ArUco 인식/집기, 서버 연동 |
| (이민정) | Mobile Robot(Nav2/AMCL), 경로/정지 제어, Localization, Goal-based Navigation |
| (김정래) | Mobile Robot(Nav2/AMCL), 경로/정지 제어, Vision(YOLO/스트리밍), 신호등 인식 |
| (김형준) | Server/FastAPI/MySQL, 서버 생성 |
| (조도훈) | Server/FastAPI/MySQL, 주문/작업 생성, |

---

## 기술 스택 (Tech Stack)

| Category | Technology |
|----------|------------|
| Development Environment | Linux (Ubuntu), Raspberry Pi, ROS 2 Jazzy |
| Language | Python |
| AI / Vision | YOLOv8 (ultralytics), OpenCV, ArUco (DICT_4X4_50) |
| Robot | Pinky (자율주행 AMR), JetCobot (로봇팔 MyCobot) |
| Navigation | Nav2 (NavigateToPose), AMCL 로컬라이제이션 |
| Server / DB | FastAPI, SQLAlchemy, MySQL |
| Camera | PiCamera2, USB Camera (12d0, 1645) |
| Hardware | Pinky LED/LCD, MyCobot 그리퍼 |
| Network | REST API, ROS 2 Topic/Action/Service, SSH |
| Collaboration | GitHub |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         사용자 (키오스크)                         │
│                              │                                   │
│                         결제 완료 →                               │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────┐            │
│  │              FastAPI 서버 (작업 생성)              │            │
│  │    주문 확정 → 로봇팔 트리거 → AMR 트리거           │            │
│  └──────────┬──────────────────────┬───────────────┘            │
│             │                      │                             │
│        ① ROS 2 토픽           ② SSH 원격 실행                   │
│             │                      │                             │
│             ▼                      ▼                             │
│  ┌──────────────────┐   ┌──────────────────────────┐           │
│  │  JetCobot 로봇팔   │   │     Pinky 자율주행 로봇     │           │
│  │                    │   │                          │           │
│  │  · ArUco 6D 인식   │   │  · Nav2 웨이포인트 주행    │           │
│  │  · Pick & Place   │   │  · AMCL 위치 추정         │           │
│  │  · 적재 완료 통보   │   │  · YOLO 신호등 인식       │           │
│  │                    │   │  · LCD/LED 상태 표시      │           │
│  └──────────────────┘   └──────────────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
```

---

## Implements

### Scenario (End-to-End)

```
① 사용자 GUI 결제 → 서버 주문 확정
     ↓
② 서버가 작업 생성 → 로봇팔 트리거 (ROS 2 / SSH)
     ↓
③ 로봇팔: ArUco 마커 인식 → 상품 집기 → Pinky에 적재
     ↓
④ 서버가 Pinky 이동 작업 트리거
     ↓
⑤ Pinky: 경로 주행 → 신호등 감지 시 정지/재개 → 목적지 도착
     ↓
⑥ LCD에 도착 안내 + LED 색상 표시
```


---

## Visual Perception

### 신호등 인식 (traffic_light_yolo_stream.py)

**YOLOv8 + HSV 이중 검증**으로 오검출을 방지합니다.

| 단계 | 처리 | 설명 |
|:---:|---|---|
| 1 | YOLOv8n 객체 탐지 | `traffic light` 클래스 검출 (conf ≥ 0.25) |
| 2 | ROI 추출 | 검출 박스 중앙 60% 영역만 사용 |
| 3 | HSV 색상 분석 | 빨강/초록 픽셀 비율 계산 |
| 4 | 연속 프레임 확인 | 빨강 1프레임, 초록 3프레임 연속 시 확정 |
| 5 | 상태 발행 | `/traffic_light_state` 토픽으로 "red"/"green" 발행 |

```
PiCamera2 → YOLOv8n → HSV 색상 판별 → /traffic_light_state 발행
                                            ↓
                              빨강 → /cmd_vel 0,0 (즉시 정지)
                              초록 → Nav2 제어 복원
```

- **Flask 웹 스트리밍:** `http://PINKY_IP:5000/video` 로 실시간 영상 확인
- **LED 연동:** 빨간불/초록불에 따라 Pinky LED 색상 자동 변경
<img width="300" height="300" alt="스크린샷 2026-02-25 12-22-47" src="https://github.com/user-attachments/assets/ec53e9c8-c5d4-4b84-9afc-9f55be888cce" />
<img width="300" height="300" alt="스크린샷 2026-02-25 12-22-59" src="https://github.com/user-attachments/assets/58833a70-f604-49ea-af3b-5ac6e834b937" />

### ArUco 마커 6D Pose 추정

| 카메라 | 스크립트 | fx | fy | cx | cy |
|---|---|---|---|---|---|
| 12d0 | step3_aruco_live.py | 861.1 | 865.2 | 397.2 | 190.9 |
| 1645 | step5_aruco_live_6d.py | 992.3 | 991.2 | 366.2 | 238.0 |

- **마커 크기:** 3cm (DICT_4X4_50)
- **출력:** 마커 ID, XYZ 위치(cm), Roll/Pitch/Yaw 각도(°)
<img width="300" height="300" alt="스크린샷 2026-02-24 15-12-03" src="https://github.com/user-attachments/assets/5ef80ba8-c5b3-46d3-bae9-8eeb6a3662b6" />
<img width="300" height="300" alt="Screenshot from 2026-02-24 11-39-02" src="https://github.com/user-attachments/assets/ec1a0a33-1609-4d2a-8043-3286dab65ee3" />

---

## Mobile Robot (Pinky)

### Nav2 자율주행

| 스크립트 | 경유 웨이포인트 | 특수 기능 |
|---|---|---|
| `waypoint_124.py` | 1 → 2 → 4 | 기본 웨이포인트 이동 |
| `route_567.py` | 5 → 6 → 7 | **신호등 인식 연동** (빨간불 자동 정지) |
| `waypoint_lcd.py` | 3개 지점 | **LCD 화면 표시 + LED 색상 제어** |

**route_567.py 신호등 브레이크:**
```
/traffic_light_state 구독
  ├─ "red"   → /cmd_vel에 0 속도 즉시 발행 (Nav2 덮어쓰기)
  └─ "green" → 정지 해제 → Nav2가 경로 제어 재개
```

**waypoint_lcd.py 동작:**
```
① LED 서버 자동 실행 (pinky_led)
② LCD에 현재 상태 표시 (이동 중 / 도착)
③ 웨이포인트 순차 이동
④ 각 지점 도착 시 LED 색상 변경
⑤ 전체 완료 시 LCD에 완료 메시지
```

### AMCL 자동 로컬라이제이션 (auto_spin_localize.py)

AMCL의 공분산이 수렴할 때까지 **자동 제자리 회전**하여 위치를 추정합니다.

| 파라미터 | 값 | 설명 |
|---|:---:|---|
| 회전 속도 | 0.6 rad/s | 제자리 회전 |
| 최대 시간 | 35초 | 타임아웃 |
| XY 공분산 임계값 | 0.05 m² | 위치 수렴 기준 |
| Yaw 공분산 임계값 | 0.20 rad² | 방향 수렴 기준 |
| 안정 시간 | 1.5초 | 임계값 유지 시간 |

```
① /reinitialize_global_localization 서비스 호출
② /cmd_vel로 회전 시작
③ /amcl_pose 구독 → 공분산 모니터링
④ XY/Yaw 공분산이 임계값 이하 1.5초 유지 → 수렴 완료, 정지
```
<img width="239" height="146" alt="스크린샷 2026-02-25 12-26-42" src="https://github.com/user-attachments/assets/50a9c441-8437-4b76-adbc-abd44302185f" /> 
<img width="239" height="146" alt="스크린샷 2026-02-25 12-26-51" src="https://github.com/user-attachments/assets/3a2d79f6-0faa-43e8-9119-d8c85e0c3b71" />

---

## Manipulator (JetCobot)

### 카메라 캘리브레이션

| 스크립트 | 기능 |
|---|---|
| `calibrate_camera.py` | 체커보드 이미지로 카메라 매트릭스, 왜곡 계수 계산 |

### ArUco 기반 Pick & Place

| 스크립트 | 대상 마커 | 기능 |
|---|---|---|
| `step1_aruco_check.py` | 6, 37번 | 다중 마커 순차 픽앤플레이스 |
| `check.py` | 7번 | 단일 마커 픽앤플레이스 테스트 |

**픽앤플레이스 동작 순서:**

```
① 시작 위치(START)로 이동
     ↓
② 카메라로 ArUco 마커 검출
   (안정 8프레임 연속 + 5초 타임아웃)
     ↓
③ 마커 ID → 해당 픽 좌표로 접근
   (위쪽 70mm → 목표 좌표)
     ↓
④ 그리퍼 닫기 (물건 잡기)
     ↓
⑤ 위로 90mm 들어올리기
     ↓
⑥ 드롭 좌표(DROP)로 이동
     ↓
⑦ 그리퍼 열기 (물건 놓기)
     ↓
⑧ 시작 위치로 복귀 → 다음 마커 반복
```

---

## ROS 2 토픽/서비스/액션

| 이름 | 타입 | 용도 |
|---|---|---|
| `/cmd_vel` | `Twist` | 로봇 속도 제어 |
| `/navigate_to_pose` | `NavigateToPose` (액션) | 웨이포인트 자율주행 |
| `/amcl_pose` | `PoseWithCovarianceStamped` | AMCL 위치 추정 결과 |
| `/traffic_light_state` | `String` | 신호등 상태 ("red"/"green") |
| `/reinitialize_global_localization` | `Empty` (서비스) | AMCL 글로벌 초기화 |
| `set_led` | `SetLed` (서비스) | Pinky LED 색상 제어 |

---

## 파일 구조 (총 11개)

```
ros2/
├── mobile_robot/                            # 자율주행 로봇 (Pinky AMR)
│   ├── navigation/
│   │   ├── waypoint_123.py                  #   웨이포인트 1→2→3
│   │   ├── waypoint_124.py                  #   웨이포인트 1→2→4 (최신 좌표)
│   │   ├── route_567.py                     #   경로 5→6→7 + 신호등 브레이크
│   │   └── waypoint_lcd.py                  #   이동 + LCD 표시 + LED 제어
│   ├── localization/
│   │   └── auto_spin_localize.py            #   AMCL 자동 회전 로컬라이제이션
│   └── vision/
│       └── traffic_light_yolo_stream.py     #   YOLOv8 신호등 + Flask 스트리밍
│
└── manipulator/                             # 로봇팔 (JetCobot)
    ├── calibration/
    │   ├── calibrate_camera.py              #   체커보드 캘리브레이션
    │   └── check.py                         #   마커 7번 픽앤플레이스 테스트
    └── aruco_detection/
        ├── step1_aruco_check.py             #   마커 6,37 픽앤플레이스
        ├── step3_aruco_live.py              #   ArUco 6D 포즈 (카메라 12d0)
        └── step5_aruco_live_6d.py           #   ArUco 6D 포즈 (카메라 1645)
```

---

## How to Run

```bash
# ── Server ──
cd server
uvicorn app.main:app --host 0.0.0.0 --port 8000

# ── Manipulator (JetCobot에서 실행) ──
# ArUco 실시간 확인
python3 ros2/manipulator/aruco_detection/step3_aruco_live.py

# 픽앤플레이스 실행
python3 ros2/manipulator/aruco_detection/step1_aruco_check.py

# ── Mobile Robot (Pinky에서 실행) ──
# 로컬라이제이션
python3 ros2/mobile_robot/localization/auto_spin_localize.py

# 신호등 인식 (별도 터미널)
python3 ros2/mobile_robot/vision/traffic_light_yolo_stream.py

# 네비게이션
python3 ros2/mobile_robot/navigation/route_567.py
```

---

## Project Schedule

- **기간:** 2025.12.29 ~ 2026.02.27

<!-- 간트차트 이미지가 있으면 아래에 넣기 -->
<!-- ![Project Schedule](docs/images/gantt_chart.png) -->

---

## 의존성

```
# ROS 2 패키지
ros2 (Jazzy), nav2_msgs, geometry_msgs, std_msgs, std_srvs
pinky_interfaces, pinky_led, pinky_lcd

# Python 패키지
opencv-python / opencv-contrib-python
numpy
pymycobot
ultralytics (YOLOv8)
picamera2
flask
pillow
fastapi, uvicorn, sqlalchemy, pymysql
```

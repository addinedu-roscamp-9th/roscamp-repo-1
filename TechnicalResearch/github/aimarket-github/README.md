# 🏪 AImarket - AI 기반 무인 자동화 마켓 키오스크 시스템

> **PyQt6 키오스크 + ROS 2 로봇팔 + 자율주행 로봇(AMR) + LLM 자연어 제어**를 통합한  
> 주문 → 결제 → 로봇 픽업 → 배송 전 과정 자동화 시스템

---

## 1. 프로젝트 개요

본 프로젝트는 **무인 마켓 환경**에서 고객의 주문부터 상품 배송까지를 완전 자동화합니다.

| 구성 요소 | 역할 | 핵심 기술 |
|---|---|---|
| **키오스크 앱** | 회원가입, 로그인, 상품 조회, 장바구니, 결제 | PyQt6, MySQL |
| **FastAPI 서버** | 주문/상품/유저 API, LLM 명령 처리 | FastAPI, SQLAlchemy |
| **LLM 라우터** | 자연어 명령 → 로봇 제어 JSON 변환 | OpenAI GPT-4o / GPT-4o-mini |
| **로봇팔 (Arm 17, 18)** | ArUco 마커 기반 상품 픽앤플레이스 | ROS 2, OpenCV, MyCobot |
| **자율주행 로봇 (Pinky)** | 상품 운반, LED/LCD 감정 표현 | ROS 2 Nav2, SSH |
| **카메라 캘리브레이션** | 로봇팔 카메라 내부 파라미터 보정 | OpenCV 체커보드 |

---

## 2. 시스템 아키텍처

```
┌──────────────────────────────────────────────────────────────┐
│                        사용자 (고객)                           │
│                            │                                  │
│                       키오스크 터치                             │
│                            ▼                                  │
│    ┌──────────────────────────────────────────┐               │
│    │       AImarket.py (PyQt6 키오스크)         │               │
│    │   · 회원가입 / 로그인    · 상품 목록         │               │
│    │   · 장바구니 / 결제      · LLM 자연어 명령   │               │
│    └───────┬──────────┬────────────┬──────────┘               │
│            │          │            │                           │
│       ① ROS 2    ② HTTP        ③ SSH                        │
│            │          │            │                           │
│            ▼          ▼            ▼                           │
│    ┌───────────┐ ┌──────────┐ ┌──────────────┐               │
│    │  로봇팔    │ │ FastAPI  │ │  Pinky AMR   │               │
│    │ Arm17/18  │ │ + LLM    │ │ 자율주행 로봇  │               │
│    └───────────┘ └──────────┘ └──────────────┘               │
└──────────────────────────────────────────────────────────────┘
```

---

## 3. 주요 기능

### 3-1. 키오스크 앱 (AImarket.py)

| 화면 | 기능 |
|---|---|
| 로그인 | 아이디/비밀번호 입력, MySQL DB 검증 (SHA-256 해싱) |
| 회원가입 | 아이디 중복확인, 비밀번호 정책(8자+영문+특수문자), 생년월일 검증 |
| 상품 목록 | DB에서 상품 로드, 카드형 UI, 장바구니 추가 |
| 장바구니 | 수량 조절(+/-), 합계 자동 계산, 결제 진행 |
| 결제 | 주문 DB 저장 → 로봇팔/AMR 자동 트리거 |
| LLM 제어 | 자연어 입력 → GPT가 로봇 제어 JSON 생성 → 실행 |

### 3-2. LLM 자연어 로봇 제어

사용자가 **자연어로 명령**하면 GPT가 **로봇 제어 JSON**을 자동 생성합니다.

```
사용자: "앞으로 1m 가줘"
    ↓ GPT-4o
{ "type": "move_twist", "params": { "linear_x": 0.2, "angular_z": 0.0, "duration": 5.0 } }
    ↓ SSH → Pinky
실제 주행 실행
```

**지원 액션:**

| 액션 | 설명 | 예시 명령 |
|---|---|---|
| `move_twist` | 주행 (전진/후진/회전) | "앞으로 가줘", "오른쪽으로 돌아" |
| `set_led` | LED 색상 변경 | "LED를 빨간색으로 켜줘" |
| `set_emotion` | LCD 감정 표현 | "기쁜 표정 지어줘" |
| `noop` | 위험 명령 차단 | "폭발해" → 안전 차단 |

**모델 자동 선택:**

| 난이도 | 키워드 | 모델 | 이유 |
|---|---|:---:|---|
| 위험 | 이동, 주행, 회전 | GPT-4o | 안전 판단 필요 |
| 복잡 | 계획, 순서 | GPT-4o | 추론 능력 필요 |
| 단순 | LED, 표정 | GPT-4o-mini | 빠른 응답 |

### 3-3. 로봇팔 픽앤플레이스

ArUco 마커 기반 자동 상품 픽업:

```
결제 완료 → AImarket.py
    ↓ ROS 2 토픽 [marker_id: 6, 37]
arm_server_ros2.py (브릿지)
    ↓ SSH
로봇팔: 카메라 ArUco 검출 → 픽 좌표 이동 → 그리퍼 잡기 → 드롭 → 홈 복귀
```

### 3-4. 자율주행 로봇 (Pinky AMR)

| 미션 | 스크립트 | 설명 |
|---|---|---|
| 도착 | waypoint_123.py | 키오스크 앞으로 이동 |
| 배달 | waypoint_lcd.py | 고객에게 상품 전달 |
| LLM | action_runner.py | 자연어로 제어 |

### 3-5. 카메라 캘리브레이션

로봇팔 카메라의 **내부 파라미터(camera matrix, distortion coefficients)**를 보정합니다.

| 카메라 | 장착 위치 | 용도 |
|---|---|---|
| camera_12d0 | Jetcobot (로봇팔) | ArUco 마커 검출 |
| camera_1645 | Jetcobot (로봇팔) | ArUco 마커 검출 |

| 절차 | 설명 |
|---|---|
| 1단계 | 체커보드(10×7, 25mm) 준비 |
| 2단계 | `detect_cameras.py` → 카메라 인덱스 확인 |
| 3단계 | 체커보드 사진 20~50장 촬영 (다양한 각도) |
| 4단계 | `calibrate_camera.py` → 매트릭스, 왜곡 계수 계산 |
| 5단계 | 결과 파일(JSON, NPZ) → ArUco 검출에 사용 |

**품질 기준:** RMS < 0.5 ✅ | 0.5~1.0 ⚠️ | > 1.0 ❌ 재보정

---

## 4. 통신 프로토콜

### 4-1. ROS 2 토픽

| 대상 | Domain ID | 토픽 | 메시지 타입 | 방향 |
|---|:---:|---|---|---|
| 로봇팔 17 | `7` | `/pick_items` | `Int32MultiArray` | 키오스크 → Arm17 |
| 로봇팔 18 | `19` | `/robot_arm_18/pick_items` | `Int32MultiArray` | 키오스크 → Arm18 |
| Pinky 이동 | `0` | `/cmd_vel` | `Twist` | action_runner → Pinky |
| Pinky LED | `0` | `set_led` (서비스) | `SetLed` | action_runner → Pinky |

> **메시지 예시:** `Int32MultiArray.data = [6, 37]` → 마커 6번, 37번 픽업 요청

### 4-2. HTTP API

| 엔드포인트 | 메서드 | 설명 |
|---|:---:|---|
| `/api/llm/command` | POST | 자연어 → 로봇 제어 JSON |
| `/api/main/*` | GET/POST | 상품, 주문, 유저 CRUD |
| `/api/robot/*` | POST | 로봇 상태 피드백 수신 |

### 4-3. SSH 원격 실행

```
AImarket.py (키오스크)
 ├─ trigger_arm17.sh  → SSH → jetcobot@ARM17 → 픽앤플레이스
 ├─ trigger_arm18.sh  → SSH → jetcobot@ARM18 → 픽앤플레이스
 ├─ trigger_pinky.sh  → SSH → pinky@PINKY   → 자율주행 미션
 └─ trigger_action.sh → SSH → pinky@PINKY   → LLM 명령 실행
```

---

## 5. LLM 처리 흐름

```
① 키오스크에서 자연어 입력
    ↓
② AImarket.py → POST /api/llm/command
    ↓
③ fastapi-server/app/api/llm_domain/router.py     (엔드포인트)
    ↓
④ app/services/llm_router.py                       (명령 분류 + GPT 호출)
   app/services/llm_service.py                      (OpenAI API 래퍼)
    ↓
⑤ OpenAI API (GPT-4o 또는 GPT-4o-mini)
    ↓
⑥ JSON 액션 반환 → AImarket.py
    ↓
⑦ trigger_action.sh → SSH → Pinky
    ↓
⑧ action_runner.py → ROS 2 (/cmd_vel, set_led)
```

---

## 6. 기술 스택

| 분류 | 기술 |
|---|---|
| **키오스크** | PyQt6 |
| **백엔드** | FastAPI, SQLAlchemy, Uvicorn |
| **데이터베이스** | MySQL (pymysql) |
| **AI/LLM** | OpenAI GPT-4o, GPT-4o-mini |
| **로봇 통신** | ROS 2 Jazzy (rclpy, std_msgs, geometry_msgs) |
| **로봇팔** | MyCobot (pymycobot), OpenCV ArUco |
| **자율주행** | ROS 2 Nav2 |
| **원격 실행** | SSH (sshpass) |
| **카메라 보정** | OpenCV 체커보드 캘리브레이션 |
| **보안** | SHA-256 해싱, 환경변수(.env) |

---

## 7. 파일 구조

```
aimarket-github/
│
├── AImarket.py                  # 메인 키오스크 앱 (PyQt6)
├── AImarketdb.py                # MySQL 테이블 초기화
│
├── scripts/                     # SSH 트리거 스크립트
│   ├── trigger_pinky.sh         #   Pinky AMR 실행
│   ├── trigger_arm17.sh         #   로봇팔 17번 실행
│   ├── trigger_arm18.sh         #   로봇팔 18번 실행
│   └── trigger_action.sh       #   LLM 명령 실행
│
├── llm-router/                  # 독립 LLM 서버
│   ├── main.py                  #   FastAPI + OpenAI
│   ├── run.sh                   #   실행 스크립트
│   ├── requirements.txt
│   └── .env.example
│
├── fastapi-server/              # FastAPI 메인 서버
│   ├── run.py                   #   서버 시작점
│   ├── .env.example
│   ├── requirements.txt
│   └── app/
│       ├── main.py              #   라우터 등록, CORS
│       ├── api/llm_domain/
│       │   └── router.py       #   /api/llm/command
│       ├── services/
│       │   ├── llm_router.py   #   명령 분류 + GPT 호출
│       │   └── llm_service.py  #   OpenAI API 래퍼
│       ├── schemas/             #   요청/응답 스키마 (llm, product, order, user, payment)
│       ├── models/              #   DB 모델 (product, order, user, payment)
│       ├── db/                  #   MySQL 세션 (session.py, base.py, init_db.py)
│       └── core/                #   설정 (config.py, logging.py)
│
├── robot/                       # 로봇 사이드 코드
│   ├── common/
│   │   ├── arm_server_ros2.py  #   ROS 2 브릿지 (서버 실행)
│   │   └── robot_arm_final.py  #   로봇팔 Flask + 픽앤플레이스
│   └── pinky/
│       └── action_runner.py    #   LLM JSON → ROS 2 실행
│
├── calibration/                 # 카메라 캘리브레이션
│   ├── calibrate_camera.py     #   체커보드 → 매트릭스 계산
│   └── detect_cameras.py       #   카메라 감지 및 테스트
│
├── run_bridges.sh               # ROS 2 브릿지 실행
├── .env.example                 # 환경변수 예시
├── .gitignore
├── requirements.txt
└── README.md
```

---

## 8. 환경변수

| 변수 | 설명 | 사용처 |
|---|---|---|
| `API_SERVER_URL` | FastAPI 서버 URL | AImarket.py |
| `DB_HOST` / `DB_PASSWORD` | MySQL 접속 정보 | AImarket.py, AImarketdb.py |
| `OPENAI_API_KEY` | OpenAI API 키 | FastAPI, llm-router |
| `PINKY_IP` / `PINKY_PASS` | Pinky SSH 접속 | trigger_pinky.sh |
| `ARM17_IP` / `ARM17_PASS` | Arm 17 SSH 접속 | trigger_arm17.sh |
| `ARM18_IP` / `ARM18_PASS` | Arm 18 SSH 접속 | trigger_arm18.sh |
| `SERVER_API_URL` | 로봇→서버 피드백 | robot_arm_final.py |

---

## 9. 실행 방법

```bash
# 1. 환경변수
cp .env.example .env              # 실제 값 입력

# 2. DB 초기화
python3 AImarketdb.py

# 3. FastAPI 서버
cd fastapi-server
cp .env.example .env              # OPENAI_API_KEY 등 설정
pip install -r requirements.txt
python3 run.py

# 4. ROS 2 브릿지 (서버)
chmod +x run_bridges.sh scripts/*.sh
./run_bridges.sh

# 5. 키오스크
python3 AImarket.py

# 6. (선택) 카메라 캘리브레이션
cd calibration
python3 detect_cameras.py         # 카메라 확인
python3 calibrate_camera.py       # 캘리브레이션 계산
```

---

## 10. 보안 처리

| 항목 | 원본 | GitHub 처리 |
|---|---|---|
| DB 비밀번호 | 하드코딩 | `os.environ.get('DB_PASSWORD')` |
| 서버 URL | 하드코딩 | `os.environ.get('API_SERVER_URL')` |
| OpenAI API 키 | `.env` 노출 | `.env.example` + `.gitignore` |
| SSH 비밀번호 | 스크립트 내 하드코딩 | `$ARM17_PASS` 등 환경변수 |
| 로봇 IP | `192.168.0.xx` 하드코딩 | `$ARM17_IP` 등 환경변수 |

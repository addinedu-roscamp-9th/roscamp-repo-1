#!/usr/bin/env python3
"""
Pinky LLM Router - GPT-5.2 / GPT-5.2 Pro 자동 스위칭 서버
로봇 제어 명령을 JSON으로 반환하는 FastAPI 기반 LLM 에이전트
"""
import os
import json
import logging
from typing import Optional, Dict, Any, List
from datetime import datetime

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from openai import OpenAI
from dotenv import load_dotenv

# 환경 변수 로드
load_dotenv()

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# OpenAI 클라이언트
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# FastAPI 앱
app = FastAPI(
    title="Pinky LLM Router",
    description="GPT-5.2 / GPT-5.2 Pro 자동 스위칭 로봇 제어 LLM",
    version="1.0.0"
)

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# === Pydantic 모델 ===
class LLMRequest(BaseModel):
    text: str = Field(..., description="사용자 명령 (자연어)")
    force_model: Optional[str] = Field(None, description="강제로 사용할 모델 (gpt-4o 또는 gpt-4o-mini)")

class ActionJSON(BaseModel):
    type: str = Field(..., description="액션 타입")
    params: Dict[str, Any] = Field(default_factory=dict, description="액션 파라미터")

class LLMResponse(BaseModel):
    model_used: str = Field(..., description="사용된 모델")
    action_json: ActionJSON = Field(..., description="파싱된 액션 JSON")
    raw_output: str = Field(..., description="LLM 원본 출력")
    reasoning: Optional[str] = Field(None, description="GPT-5.2 Pro의 추론 과정")
    classification: str = Field(..., description="명령 분류 (simple/complex/dangerous)")


# === 모델 선택 로직 ===
def classify_command(text: str) -> str:
    """
    명령을 분류합니다.
    - simple: 단순 명령 (LED, LCD, 설명)
    - complex: 복잡한 계획 필요
    - dangerous: 위험한 동작 (주행, 이동)
    """
    text_lower = text.lower()
    
    # 위험/복잡한 키워드 (주행 관련)
    dangerous_keywords = [
        "가라", "이동", "주행", "돌아", "움직", "전진", "후진",
        "nav2", "navigation", "slam", "맵", "경로",
        "follow", "따라", "회전", "rotate", "turn"
    ]
    
    # 복잡한 계획 키워드
    complex_keywords = [
        "계획", "순서", "먼저", "그다음", "한 바퀴",
        "찾아서", "확인하고", "저장하고"
    ]
    
    # 단순 명령 키워드
    simple_keywords = [
        "led", "엘이디", "불", "빛", "색",
        "lcd", "감정", "표정", "이모티콘",
        "설명", "알려줘", "뭐야", "무엇"
    ]
    
    # 우선순위: dangerous > complex > simple
    if any(k in text_lower for k in dangerous_keywords):
        return "dangerous"
    elif any(k in text_lower for k in complex_keywords):
        return "complex"
    elif any(k in text_lower for k in simple_keywords):
        return "simple"
    else:
        # 기본적으로 안전을 위해 complex로 분류
        return "complex"


def choose_model(text: str, force_model: Optional[str] = None) -> str:
    """
    명령을 분석하여 적절한 모델을 선택합니다.
    
    규칙:
    - dangerous/complex → gpt-4o (고난도 추론 필요)
    - simple → gpt-4o-mini (빠른 응답)
    """
    if force_model:
        return force_model
    
    classification = classify_command(text)
    
    if classification in ["dangerous", "complex"]:
        return "gpt-4o"  # 고성능 모델
    else:
        return "gpt-4o-mini"  # 빠른 응답용


# === 시스템 프롬프트 ===
SYSTEM_PROMPT = """
너는 교육용 이동 로봇 Pinky Pro를 제어하는 LLM 에이전트다.

**중요: 반드시 JSON 형식으로만 답변해라. 다른 설명 문장은 절대 넣지 마라.**

JSON 스키마:

```json
{{
  "type": "<action_type>",
  "params": {{
    ...
  }}
}}
```

**action_type 종류:**

1. **move_twist**: /cmd_vel로 주행 (직진/회전)
   params:
   - linear_x: float (m/s, -0.3~0.3 범위)
   - angular_z: float (rad/s, -1.0~1.0 범위)
   - duration: float (초, 0.1~5.0 범위)

2. **set_led**: LED 전체 색상 설정
   params:
   - r: int (0~255)
   - g: int (0~255)
   - b: int (0~255)

3. **set_emotion**: LCD 감정 표시
   params:
   - emotion: string (예: "hello", "happy", "sad", "angry", "surprised", "normal")

4. **noop**: 아무 행동도 하지 않음
   params: {{}}

**안전 규칙:**
- linear_x: -0.3 ~ 0.3 m/s 범위 엄수
- angular_z: -1.0 ~ 1.0 rad/s 범위 엄수
- duration: 0.1 ~ 5.0초 범위 엄수
- 위험하거나 모호한 명령은 type을 "noop"으로 설정
- 장애물 회피나 안전 확인이 필요한 경우 속도를 절반으로 줄여라

**예시:**

사용자: "앞으로 1m 가줘"
응답:
```json
{{
  "type": "move_twist",
  "params": {{
    "linear_x": 0.2,
    "angular_z": 0.0,
    "duration": 5.0
  }}
}}
```

사용자: "LED를 빨간색으로 켜줘"
응답:
```json
{{
  "type": "set_led",
  "params": {{
    "r": 255,
    "g": 0,
    "b": 0
  }}
}}
```

사용자: "기쁜 표정 지어줘"
응답:
```json
{{
  "type": "set_emotion",
  "params": {{
    "emotion": "happy"
  }}
}}
```

현재 날짜: {current_date}

사용자 명령을 보고 가장 적절한 하나의 action만 선택해서 JSON으로 반환하라.
"""


# === LLM 호출 ===
def call_llm(text: str, model: str) -> tuple[str, Optional[str]]:
    """
    LLM을 호출하고 응답을 반환합니다.
    
    Returns:
        (output_text, reasoning_text or None)
    """
    current_date = datetime.now().strftime("%Y-%m-%d %A")
    
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT.format(current_date=current_date)},
        {"role": "user", "content": text}
    ]
    
    try:
        # GPT-4o와 GPT-4o-mini 모두 동일한 API 사용
        # gpt-4o는 temperature를 더 낮춰서 더 일관성 있는 출력
        if model == "gpt-4o":
            temperature = 0.1
        else:
            temperature = 0.3
        
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            temperature=temperature,
        )
        
        output_text = response.choices[0].message.content
        reasoning_text = None  # reasoning은 현재 지원 안 됨
        
        logger.info(f"Model: {model}, Input: {text[:50]}..., Output length: {len(output_text)}")
        return output_text, reasoning_text
        
    except Exception as e:
        logger.error(f"LLM call failed: {e}")
        raise HTTPException(status_code=500, detail=f"LLM API 호출 실패: {str(e)}")


# === API 엔드포인트 ===
@app.get("/")
def root():
    return {
        "service": "Pinky LLM Router",
        "version": "1.0.0",
        "status": "running"
    }


@app.post("/llm", response_model=LLMResponse)
def llm_endpoint(req: LLMRequest):
    """
    자연어 명령을 받아서 로봇 제어 JSON을 반환합니다.
    """
    logger.info(f"Request: {req.text}")
    
    # 1. 명령 분류 및 모델 선택
    classification = classify_command(req.text)
    model = choose_model(req.text, req.force_model)
    
    logger.info(f"Classification: {classification}, Model: {model}")
    
    # 2. LLM 호출
    output_text, reasoning_text = call_llm(req.text, model)
    
    # 3. JSON 파싱
    try:
        # JSON 블록 추출 (```json ... ``` 형태 처리)
        if "```json" in output_text:
            start = output_text.find("```json") + 7
            end = output_text.find("```", start)
            json_str = output_text[start:end].strip()
        elif "```" in output_text:
            start = output_text.find("```") + 3
            end = output_text.find("```", start)
            json_str = output_text[start:end].strip()
        else:
            json_str = output_text.strip()
        
        action_dict = json.loads(json_str)
        action_json = ActionJSON(**action_dict)
        
    except json.JSONDecodeError as e:
        logger.error(f"JSON parse failed: {e}, Output: {output_text[:200]}")
        raise HTTPException(
            status_code=500,
            detail=f"LLM 응답이 유효한 JSON이 아닙니다: {str(e)}"
        )
    except Exception as e:
        logger.error(f"Action validation failed: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"액션 검증 실패: {str(e)}"
        )
    
    # 4. 응답 반환
    return LLMResponse(
        model_used=model,
        action_json=action_json,
        raw_output=output_text,
        reasoning=reasoning_text,
        classification=classification
    )


@app.get("/health")
def health_check():
    """헬스 체크"""
    return {"status": "healthy", "timestamp": datetime.now().isoformat()}


if __name__ == "__main__":
    import uvicorn
    
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)

"""
LLM Router Service - 로봇 제어용 LLM 라우팅
자연어 명령을 로봇 제어 JSON으로 변환
"""
import json
import logging
from typing import Optional, Dict, Any, Tuple
from datetime import datetime
from openai import OpenAI

logger = logging.getLogger(__name__)


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


class LLMRouter:
    """로봇 제어용 LLM 라우터"""
    
    def __init__(self, api_key: Optional[str] = None):
        """
        Args:
            api_key: OpenAI API 키
        """
        # Settings에서 로드
        if api_key is None:
            from app.core.config import settings
            api_key = settings.OPENAI_API_KEY
        
        if not api_key:
            raise ValueError("OPENAI_API_KEY not found")
        
        self.api_key = api_key
        self.client = OpenAI(api_key=self.api_key)
        self.enabled = True
        logger.info("LLM Router initialized")
    
    def classify_command(self, text: str) -> str:
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
    
    def choose_model(self, text: str, force_model: Optional[str] = None) -> str:
        """
        명령을 분석하여 적절한 모델을 선택합니다.
        
        규칙:
        - dangerous/complex → gpt-4o (고난도 추론 필요)
        - simple → gpt-4o-mini (빠른 응답)
        """
        if force_model:
            return force_model
        
        classification = self.classify_command(text)
        
        if classification in ["dangerous", "complex"]:
            return "gpt-4o"  # 고성능 모델
        else:
            return "gpt-4o-mini"  # 빠른 응답용
    
    def call_llm(self, text: str, model: str) -> Tuple[str, Optional[str]]:
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
            
            response = self.client.chat.completions.create(
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
            raise Exception(f"LLM API 호출 실패: {str(e)}")
    
    def parse_action_json(self, output_text: str) -> Dict[str, Any]:
        """LLM 출력에서 JSON 추출 및 파싱"""
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
            return action_dict
            
        except json.JSONDecodeError as e:
            logger.error(f"JSON parse failed: {e}, Output: {output_text[:200]}")
            raise Exception(f"LLM 응답이 유효한 JSON이 아닙니다: {str(e)}")
    
    def route(
        self,
        text: str,
        force_model: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        자연어 명령을 로봇 제어 JSON으로 변환
        
        Args:
            text: 사용자 명령 (자연어)
            force_model: 강제로 사용할 모델 (gpt-4o 또는 gpt-4o-mini)
        
        Returns:
            {
                "model_used": str,
                "action_json": dict,
                "raw_output": str,
                "reasoning": str or None,
                "classification": str
            }
        """
        if not self.enabled:
            raise Exception("LLM Router가 비활성화되어 있습니다")
        
        # 1. 명령 분류 및 모델 선택
        classification = self.classify_command(text)
        model = self.choose_model(text, force_model)
        
        logger.info(f"Request: {text}, Classification: {classification}, Model: {model}")
        
        # 2. LLM 호출
        output_text, reasoning_text = self.call_llm(text, model)
        
        # 3. JSON 파싱
        action_json = self.parse_action_json(output_text)
        
        # 4. 결과 반환
        return {
            "model_used": model,
            "action_json": action_json,
            "raw_output": output_text,
            "reasoning": reasoning_text,
            "classification": classification
        }


# 싱글톤 인스턴스 (lazy loading)
_llm_router_instance: Optional[LLMRouter] = None


def get_llm_router() -> LLMRouter:
    """LLM Router 싱글톤 인스턴스 반환"""
    global _llm_router_instance
    if _llm_router_instance is None:
        _llm_router_instance = LLMRouter()
    return _llm_router_instance

"""
LLM API 스키마
"""
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any


class LLMRequest(BaseModel):
    """LLM 요청"""
    text: str = Field(..., description="사용자 명령 (자연어)")
    force_model: Optional[str] = Field(None, description="강제로 사용할 모델 (gpt-4o 또는 gpt-4o-mini)")


class ActionJSON(BaseModel):
    """로봇 제어 액션 JSON"""
    type: str = Field(..., description="액션 타입")
    params: Dict[str, Any] = Field(default_factory=dict, description="액션 파라미터")


class LLMResponse(BaseModel):
    """LLM 응답"""
    model_used: str = Field(..., description="사용된 모델")
    action_json: ActionJSON = Field(..., description="파싱된 액션 JSON")
    raw_output: str = Field(..., description="LLM 원본 출력")
    reasoning: Optional[str] = Field(None, description="GPT-4o의 추론 과정")
    classification: str = Field(..., description="명령 분류 (simple/complex/dangerous)")

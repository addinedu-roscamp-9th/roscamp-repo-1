"""
LLM Domain Router
"""
from fastapi import APIRouter, HTTPException
from app.schemas.llm import LLMRequest, LLMResponse, ActionJSON
from app.services.llm_router import get_llm_router
import logging

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/command", response_model=LLMResponse)
async def llm_command(request: LLMRequest):
    """
    자연어 명령을 로봇 제어 JSON으로 변환
    
    - **text**: 사용자 명령 (예: "LED를 빨간색으로 켜줘")
    - **force_model**: 강제로 사용할 모델 (선택)
    
    Returns:
        - model_used: 사용된 GPT 모델
        - action_json: 파싱된 로봇 제어 JSON
        - classification: 명령 분류
    """
    try:
        llm_router = get_llm_router()
        result = llm_router.route(request.text, request.force_model)
        
        return LLMResponse(
            model_used=result["model_used"],
            action_json=ActionJSON(**result["action_json"]),
            raw_output=result["raw_output"],
            reasoning=result["reasoning"],
            classification=result["classification"]
        )
    except Exception as e:
        logger.error(f"LLM command failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health")
async def llm_health():
    """LLM 서비스 상태 확인"""
    try:
        llm_router = get_llm_router()
        return {
            "status": "healthy" if llm_router.enabled else "disabled",
            "service": "LLM Router",
            "models": ["gpt-4o", "gpt-4o-mini"]
        }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e)
        }

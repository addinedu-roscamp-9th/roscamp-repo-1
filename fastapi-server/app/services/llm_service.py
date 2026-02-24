"""
LLM Service - GPT API 호출 및 프롬프트 관리
운영자의 자연어 질의를 DB 데이터와 함께 GPT에 전달하여 답변 생성
"""
import openai
from typing import Dict, List, Optional
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)


class LLMService:
    """GPT API 호출 서비스"""
    
    def __init__(self):
        # OpenAI API 키 설정
        self.api_key = getattr(settings, "OPENAI_API_KEY", None)
        if self.api_key:
            openai.api_key = self.api_key
        self.model = getattr(settings, "OPENAI_MODEL", "gpt-4")
        self.enabled = bool(self.api_key)
    
    async def query_with_context(
        self,
        user_query: str,
        context_data: Dict,
        system_prompt: Optional[str] = None
    ) -> Dict:
        """
        사용자 질의 + DB 컨텍스트를 GPT에 전달하여 답변 생성
        
        Args:
            user_query: 사용자의 자연어 질문
            context_data: DB에서 가져온 데이터 (robot_status, logs, tasks 등)
            system_prompt: 시스템 프롬프트 (선택)
        
        Returns:
            {
                "success": bool,
                "answer": str,
                "tokens_used": int,
                "model": str
            }
        """
        if not self.enabled:
            return {
                "success": False,
                "answer": "LLM 서비스가 비활성화되어 있습니다. OpenAI API 키를 설정하세요.",
                "error": "API key not configured"
            }
        
        try:
            # 기본 시스템 프롬프트
            if system_prompt is None:
                system_prompt = """
당신은 로봇 운영 어시스턴트입니다.
주어진 데이터를 기반으로만 정확하게 답변하세요.
추측하지 말고, 데이터가 없으면 "데이터가 없습니다"라고 답변하세요.
모든 답변은 한글로 작성하세요.
"""
            
            # 사용자 프롬프트 생성 (질문 + 컨텍스트)
            user_prompt = self._build_user_prompt(user_query, context_data)
            
            # GPT API 호출
            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,  # 낮은 temperature로 일관된 답변
                max_tokens=800
            )
            
            answer = response.choices[0].message.content
            tokens_used = response.usage.total_tokens
            
            logger.info(f"LLM query successful: {tokens_used} tokens used")
            
            return {
                "success": True,
                "answer": answer,
                "tokens_used": tokens_used,
                "model": self.model
            }
        
        except Exception as e:
            logger.error(f"LLM query failed: {e}")
            return {
                "success": False,
                "answer": f"답변 생성 중 오류가 발생했습니다: {str(e)}",
                "error": str(e)
            }
    
    def _build_user_prompt(self, query: str, context_data: Dict) -> str:
        """사용자 프롬프트 생성 (질문 + DB 데이터)"""
        prompt_parts = [f"사용자 질문: {query}\n"]
        
        # Robot Status 데이터
        if "robot_status" in context_data and context_data["robot_status"]:
            prompt_parts.append("\n### 로봇 상태 데이터:")
            for robot in context_data["robot_status"]:
                prompt_parts.append(
                    f"- {robot.get('robot_id')}: "
                    f"상태={robot.get('status')}, "
                    f"배터리={robot.get('battery_level')}%, "
                    f"위치=({robot.get('current_x')}, {robot.get('current_y')})"
                )
        
        # Logs 데이터
        if "logs" in context_data and context_data["logs"]:
            prompt_parts.append("\n### 최근 로그 데이터:")
            for log in context_data["logs"][:10]:  # 최근 10개
                prompt_parts.append(
                    f"- [{log.get('level')}] {log.get('source')}: {log.get('message')}"
                )
        
        # Tasks 데이터
        if "tasks" in context_data and context_data["tasks"]:
            prompt_parts.append("\n### 작업 데이터:")
            for task in context_data["tasks"]:
                prompt_parts.append(
                    f"- Task {task.get('id')}: "
                    f"{task.get('task_type')} - {task.get('status')}"
                )
        
        prompt_parts.append("\n위 데이터를 기반으로 질문에 답변해주세요.")
        
        return "\n".join(prompt_parts)


# 싱글톤 인스턴스
llm_service = LLMService()

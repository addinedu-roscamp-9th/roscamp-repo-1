#!/bin/bash
set -e

echo "======================================"
echo " Pinky LLM Router 서버 시작"
echo "======================================"

# 가상환경 확인 및 생성
if [ ! -d "venv" ]; then
    echo "📦 가상환경 생성 중..."
    python3 -m venv venv
fi

# 가상환경 활성화
echo "✅ 가상환경 활성화"
source venv/bin/activate

# 패키지 설치
echo "📥 패키지 설치/업데이트 중..."
pip install -q --upgrade pip
pip install -q -r requirements.txt

# .env 파일 확인
if [ ! -f ".env" ]; then
    echo "⚠️  .env 파일이 없습니다. .env.example을 복사하여 .env를 만들고 API 키를 설정하세요."
    exit 1
fi

# 서버 실행
echo ""
echo "🚀 서버 시작..."
echo "======================================"
python3 main.py

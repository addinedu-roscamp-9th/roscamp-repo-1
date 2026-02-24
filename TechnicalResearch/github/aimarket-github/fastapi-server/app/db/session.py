from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base
import os

# 환경변수에서 DATABASE_URL을 읽어옵니다.
# .env 파일 예시: DATABASE_URL=mysql+pymysql://root:password@127.0.0.1:3306/fastapi_db
DATABASE_URL = os.environ.get(
    "DATABASE_URL",
    "mysql+pymysql://root:password@127.0.0.1:3306/fastapi_db"
)

engine = create_engine(DATABASE_URL)

SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine
)

Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field
from typing import List

class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

    APP_NAME: str = Field(default="FastAPI Basic Server")
    ENV: str = Field(default="dev")
    LOG_LEVEL: str = Field(default="INFO")

    # CORS: comma-separated or '*'
    CORS_ORIGINS: str = Field(default="*")

    # DB - MySQL connection string
    # Format: mysql+pymysql://username:password@host:port/database_name
    DATABASE_URL: str = Field(default="mysql+pymysql://root:password@localhost:3306/fastapi_db")
    
    # Control Tower Server
    CONTROL_TOWER_URL: str = Field(default="http://localhost:8001")
    CONTROL_TOWER_ENABLED: bool = Field(default=True)
    
    # OpenAI / LLM
    OPENAI_API_KEY: str = Field(default="")
    OPENAI_MODEL: str = Field(default="gpt-4")

    @property
    def CORS_ORIGINS_LIST(self) -> List[str]:
        value = (self.CORS_ORIGINS or "").strip()
        if value == "*" or value == "":
            return ["*"]
        return [v.strip() for v in value.split(",") if v.strip()]

settings = Settings()

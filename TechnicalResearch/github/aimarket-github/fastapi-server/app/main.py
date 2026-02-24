from fastapi import FastAPI
from fastapi.responses import RedirectResponse
from fastapi.middleware.cors import CORSMiddleware



from app.core.config import settings
from app.core.logging import configure_logging

# Domain routers
from app.api.main_domain import main_router
from app.api.control_domain import control_router
from app.api.llm_domain import llm_router
from app.api.robot_domain import robot_router

configure_logging()

app = FastAPI(
    title=settings.APP_NAME,
    version="2.0.0",
    description="Main Server with Domain Separation: Main Domain (Business) + Control Domain (Monitoring) + LLM Domain (Robot Control) + Robot Domain (ArUco/Robots)",
)

# CORS
origins = settings.CORS_ORIGINS_LIST
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Domain Routes
app.include_router(main_router, prefix="/api/main")
app.include_router(control_router, prefix="/api/control")
app.include_router(llm_router, prefix="/api/llm", tags=["llm"])
app.include_router(robot_router, prefix="/api/robot", tags=["robot"])


# Static Files
from fastapi.staticfiles import StaticFiles
import os

# Check if static directory exists
static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    app.mount("/web", StaticFiles(directory=static_dir, html=True), name="static")

# Root Directory Redirect
@app.get("/", include_in_schema=False)
async def root():
    return RedirectResponse(url="/web/index.html")


# Root health check
@app.get("/health", tags=["health"])

def health():
    return {
        "status": "ok",
        "env": settings.ENV,
        "version": "2.0.0",
        "domains": {
            "main": "/api/main/*",
            "control": "/api/control/*",
            "llm": "/api/llm/*",
            "robot": "/api/robot/*"
        }
    }

# Legacy v1 API redirect info (optional)
@app.get("/api/v1", tags=["info"])
def v1_info():
    return {
        "message": "API v1 has been reorganized into domain structure",
        "new_structure": {
            "main_domain": "/api/main/*",
            "control_domain": "/api/control/*",
            "llm_domain": "/api/llm/*",
            "robot_domain": "/api/robot/*"
        },
        "docs": "/docs"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)


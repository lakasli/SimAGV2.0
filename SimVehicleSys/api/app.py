from __future__ import annotations

"""API 入口封装：复用 backend.main.app，并挂载 SimVehicleSys 路由，实现统一 API 层。"""

try:
    from backend.main import app as backend_app
except Exception:
    backend_app = None  # type: ignore

try:
    from .routes import router as sim_router
except Exception:
    sim_router = None  # type: ignore


def get_app():
    if not backend_app:
        raise RuntimeError("backend FastAPI app unavailable")
    try:
        if sim_router:
            backend_app.include_router(sim_router)
    except Exception:
        pass
    return backend_app
# main.py
from fastapi import FastAPI, APIRouter
import uvicorn
from fastapi.middleware.cors import CORSMiddleware
from .ros2_bridge import start_ros2, stop_ros2, get_ros2_node
from .routes.ws_router import joy_router, register_ros2_callback
from .routes.ws_img import img_router

app = FastAPI()
api = APIRouter()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
 #routes
api.include_router(joy_router, prefix="/connection", tags=["connection"])
api.include_router(img_router, prefix="/connection", tags=["connection"])


#includes
app.include_router(api, prefix= "/ws")

@app.on_event("startup")
async def startup():
    start_ros2()
    register_ros2_callback()

@app.on_event("shutdown")
async def shutdown():
    stop_ros2()

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
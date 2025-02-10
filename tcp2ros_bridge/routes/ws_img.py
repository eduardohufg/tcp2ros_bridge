# routes/ws_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio
from ..ros2_bridge import get_ros2_node
import cv2
import base64
import numpy as np
from fastapi.responses import HTMLResponse


img_router = APIRouter()
active_connections: Set[WebSocket] = set()

@img_router.websocket("/img")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    cap = cv2.VideoCapture(0)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            _,buffer = cv2.imencode(".jpg", frame)
            frame_base64 = base64.b64encode(buffer).decode("utf-8")

            await websocket.send_text(frame_base64)
            await asyncio.sleep(0.033)
    except WebSocketDisconnect:
        print("client disconnect")

    finally:
        cap.release()
# routes/ws_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio
from ..ros2_bridge import get_ros2_node  

joy_router = APIRouter()
active_connections: Set[WebSocket] = set()

# Función para enviar mensajes a los WebSockets
async def send_to_websockets(message: str):
    for websocket in active_connections:
        try:
            await websocket.send_text(f"ROS2: {message}")
        except Exception as e:
            print(f"Error sending message: {e}")

# Registra el callback después de que ros2_node esté inicializado
def register_ros2_callback():
    ros2_node = get_ros2_node()
    if ros2_node is not None:
        ros2_node.register_callback(lambda msg: asyncio.run(send_to_websockets(msg)))
    else:
        raise RuntimeError("ROS2 node no initialized")

@joy_router.websocket("/joy")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            ros2_node = get_ros2_node()
            if ros2_node is not None:
                ros2_node.publish_message(data)
                await websocket.send_text(f"Tú: {data}")
            else:
                await websocket.send_text("Error: ROS2 node no available")
    except WebSocketDisconnect:
        active_connections.remove(websocket)
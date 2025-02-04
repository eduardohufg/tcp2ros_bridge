import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn
import asyncio
from threading import Thread
from typing import Set

app = FastAPI()

# Almacena conexiones WebSocket activas
active_connections: Set[WebSocket] = set()

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')
        
        # Publisher para enviar mensajes a ROS 2
        self.publisher = self.create_publisher(String, 'command_topic', 10)
        
        # Subscriber para recibir mensajes de ROS 2
        self.subscription = self.create_subscription(
            String,
            'response_topic',
            self.listener_callback,
            10
        )
        
    def publish_message(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicado en ROS 2: "{msg.data}"')
    
    def listener_callback(self, msg: String):
        self.get_logger().info(f'Recibido de ROS 2: "{msg.data}"')
        
        # Envía el mensaje a todos los clientes WebSocket conectados
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        for websocket in active_connections:
            try:
                loop.run_until_complete(
                    websocket.send_text(f"ROS2 dice: {msg.data}")
                )
            except Exception as e:
                print(f"Error enviando mensaje: {e}")

ros2_node = None
executor = None

@app.on_event("startup")
async def startup_event():
    global ros2_node, executor
    
    # Inicializa ROS 2
    rclpy.init()
    ros2_node = ROS2Bridge()
    
    # Ejecuta el executor de ROS 2 en un hilo separado
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros2_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

@app.on_event("shutdown")
async def shutdown_event():
    global ros2_node, executor
    executor.shutdown()
    ros2_node.destroy_node()
    rclpy.shutdown()

@app.websocket("/ws/")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    
    try:
        while True:
            # Recibe mensajes del cliente WebSocket
            data = await websocket.receive_text()
            
            # Publica el mensaje en ROS 2
            ros2_node.publish_message(data)
            await websocket.send_text(f"Tú: {data}")
            
    except WebSocketDisconnect:
        active_connections.remove(websocket)
        print("Cliente desconectado")

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()
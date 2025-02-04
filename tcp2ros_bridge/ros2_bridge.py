# ros2_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
from threading import Thread
from typing import List, Callable

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')
        self.publisher = self.create_publisher(String, 'command_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'response_topic',
            self.listener_callback,
            10
        )
        self._callbacks: List[Callable[[str], None]] = []

    def publish_message(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

    def listener_callback(self, msg: String):
        self.get_logger().info(f'Received: "{msg.data}"')
        for callback in self._callbacks:
            callback(msg.data)

    def register_callback(self, callback: Callable[[str], None]):
        self._callbacks.append(callback)

ros2_node: ROS2Bridge = None 
executor = None

def start_ros2():
    global ros2_node, executor
    rclpy.init()
    ros2_node = ROS2Bridge()  # Inicializa ros2_node aquí
    executor = SingleThreadedExecutor()
    executor.add_node(ros2_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

def stop_ros2():
    global ros2_node, executor
    executor.shutdown()
    ros2_node.destroy_node()
    rclpy.shutdown()

def get_ros2_node() -> ROS2Bridge:
    return ros2_node  # Función para acceder a ros2_node de forma segura
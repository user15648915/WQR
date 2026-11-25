#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_wheels_node')
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.start_time = time.time()
        self.get_logger().info('Script started: Waiting 5 seconds for robot to stand up...')

    def publish_cmd(self):
        msg = Float64MultiArray()
        # Ждем 5 секунд
        if time.time() - self.start_time > 5.0:
            # ПОЛОЖИТЕЛЬНЫЕ значения = движение НАЗАД. 
            # 5.0 = плавная, спокойная скорость (без рывков на дыбы)
            msg.data = [-5.0, -5.0, -5.0, -5.0]
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

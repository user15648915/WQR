#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge
import time

class BrainNode(Node):
    def __init__(self):
        super().__init__('smart_brain_node')
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.bridge = CvBridge()
        self.start_time = time.time()
        
        # ПАМЯТЬ РОБОТА: текущее состояние
        self.state = 'FORWARD' 
        self.get_logger().info('BRAIN ONLINE: State Machine initialized.')

    def camera_callback(self, data):
        if time.time() - self.start_time < 5.0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = (40, 50, 50)
        upper_green = (80, 255, 255)
        mask = cv2.inRange(hsv, lower_green, upper_green)

        green_ratio = cv2.countNonZero(mask) / mask.size
        msg = Float64MultiArray()

        # УМНАЯ ЛОГИКА С ГИСТЕРЕЗИСОМ
        if self.state == 'FORWARD':
            if green_ratio > 0.10: # Подошли слишком близко
                self.get_logger().info(f'OBSTACLE DETECTED ({green_ratio*100:.1f}%)! Switching to REVERSE.')
                self.state = 'REVERSING'
                msg.data = [5.0, 5.0, 5.0, 5.0]
            else:
                msg.data = [-5.0, -5.0, -5.0, -5.0] # Продолжаем ехать вперед
        
        elif self.state == 'REVERSING':
            if green_ratio < 0.02: # Отъехали далеко назад (зеленого почти не видно)
                self.get_logger().info(f'SAFE DISTANCE REACHED ({green_ratio*100:.1f}%). Switching to FORWARD.')
                self.state = 'FORWARD'
                msg.data = [-5.0, -5.0, -5.0, -5.0]
            else:
                msg.data = [5.0, 5.0, 5.0, 5.0] # Продолжаем отступать

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = BrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

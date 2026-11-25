#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class DefaultPostureNode(Node):
    def __init__(self):
        super().__init__('default_posture_node')
        self.leg_pub = self.create_publisher(Float64MultiArray, '/leg_controller/commands', 10)
        # Частота удержания стойки 50 Гц
        self.timer = self.create_timer(0.02, self.publish_posture)
        self.get_logger().info('Default Posture: Standing locked at 0.0 rad.')

    def publish_posture(self):
        # 12 приводов ног жестко удерживают нулевую стойку
        leg_msg = Float64MultiArray()
        leg_msg.data = [0.0] * 12
        self.leg_pub.publish(leg_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DefaultPostureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

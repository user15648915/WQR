#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SafeStandNode(Node):
    def __init__(self):
        super().__init__('leg_command_bridge')
        self.leg_pub = self.create_publisher(JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.3, self.tick)
        self.phase = 0

    def tick(self):
        if self.leg_pub.get_subscription_count() == 0:
            return
        if self.phase == 0:
            # Ждем 0.3 сек после появления подписчика для надежной активации
            self.phase = 1
        elif self.phase == 1:
            msg = JointTrajectory()
            msg.joint_names = [
                'fl_hip_joint', 'fl_thigh_joint', 'fl_shank_joint',
                'fr_hip_joint', 'fr_thigh_joint', 'fr_shank_joint',
                'rl_hip_joint', 'rl_thigh_joint', 'rl_shank_joint',
                'rr_hip_joint', 'rr_thigh_joint', 'rr_shank_joint'
            ]
            point = JointTrajectoryPoint()
            point.positions = [
                0.0, 0.5, -1.0, 0.0, 0.5, -1.0,
                0.0, 0.5, -1.0, 0.0, 0.5, -1.0
            ]
            point.time_from_start.sec = 1
            point.time_from_start.nanosec = 0
            msg.points = [point]
            self.leg_pub.publish(msg)
            self.phase = 2
        elif self.phase < 6:
            self.phase += 1
        else:
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SafeStandNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        self.pub = self.create_publisher(TwistStamped, '/skidbot_controller/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._cb, 10)

    def _cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(TwistToTwistStamped())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

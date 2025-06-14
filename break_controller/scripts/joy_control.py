#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')

        # === Settings ===
        self.v_max         = 2.5                        # max linear speed (m/s)
        self.w_max         = np.tan(0.698) / 0.263      # max angular speed (rad/s)
        self.deadzone      = 0.05                       # joystick deadzone
        self.enable_button = 6                          # only move while this is held

        # === ROS interfaces ===
        self.sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )
        self.pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        # reuse one Twist message
        self.twist = Twist()

    @staticmethod
    def _clamp(x, low: float = -1.0, high: float = 1.0) -> float:
        """Clamp x to the interval [low, high]."""
        return max(low, min(high, x))

    def _apply_deadzone(self, x: float) -> float:
        """Zero out small joystick noise around zero."""
        return x if abs(x) > self.deadzone else 0.0

    def joy_callback(self, msg: Joy):
        # deadman switch: only drive while button 6 is held
        pressed = (
            len(msg.buttons) > self.enable_button
            and msg.buttons[self.enable_button] == 1
        )

        if pressed and len(msg.axes) > 2:
            # read & sanitize raw inputs
            raw_fwd   = msg.axes[1]        # already filtered by joy_node
            raw_steer = msg.axes[2]

            # # apply deadzone
            # raw_fwd   = self._apply_deadzone(raw_fwd)
            # raw_steer = self._apply_deadzone(raw_steer)

            # scale → real-world commands
            self.twist.linear.x  = raw_fwd  * self.v_max
            self.twist.angular.z = raw_steer * self.w_max
        else:
            # zero out to ensure robot stops
            self.twist.linear.x  = 0.0
            self.twist.angular.z = 0.0

        # always publish so the controller receives stop commands too
        self.pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

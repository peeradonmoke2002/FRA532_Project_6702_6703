#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8

class BreakController(Node):
    def __init__(self):
        super().__init__('break_controller')
        # subscribe to the raw /joy topic
        self.create_subscription(Joy, 'joy', self.cb_joy, 10)
        # publishers for your two topics
        self.pub_mode = self.create_publisher(Bool, 'mode', 10)
        self.pub_pwm  = self.create_publisher(UInt8, 'pwm_duty', 10)
        # new: publisher to cmd_vel
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # pre-create a zero-Twist to avoid reallocating
        self._zero_twist = Twist()
        self._zero_twist.linear.x = 0.0
        self._zero_twist.linear.y = 0.0
        self._zero_twist.linear.z = 0.0
        self._zero_twist.angular.x = 0.0
        self._zero_twist.angular.y = 0.0
        self._zero_twist.angular.z = 0.0

    def cb_joy(self, msg: Joy):
        # BUTTON 6 hold → mode on/off
        mode = Bool(data=bool(msg.buttons[6]))
        self.pub_mode.publish(mode)

        # read L2 trigger: raw in [-1…1], default=1, pressed→-1
        raw = msg.axes[5]

        if mode.data:
            # invert so unpressed→0, full press→255
            duty_val = int((1.0 - raw) * 0.5 * 255)
            # braking engaged → immediately stop robot
            self.pub_cmd_vel.publish(self._zero_twist)
        else:
            duty_val = 0

        pwm = UInt8(data=duty_val)
        self.pub_pwm.publish(pwm)

        self.get_logger().debug(
            f'mode={mode.data}, axis5={raw:.2f}, duty={duty_val}'
        )

def main():
    rclpy.init()
    node = BreakController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

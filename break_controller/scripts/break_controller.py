#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8

class BreakController(Node):
    def __init__(self):
        super().__init__('break_controller')

        # === Configuration ===
        self.enable_button = 6      # hold to enable braking input (button index)
        self.trigger_axis  = 5      # L2 trigger (axis index)
        self.max_duty      = 255    # full-scale PWM
        self.duty_scale    = 1      # fraction of max_duty

        # === ROS interfaces ===
        # 1) Joystick input
        self.create_subscription(Joy,              '/joy',            self.cb_joy,     10)
        # 2) Teleop cmd_vel (coming from your teleop node)
        self.create_subscription(Twist,            '/teleop_cmd_vel', self.cb_cmd_vel, 10)
        # 3) Publishers
        self.pub_mode       = self.create_publisher(Bool,   '/break_mode', 10)
        self.pub_pwm        = self.create_publisher(UInt8,  '/break_pwm',  10)
        self.pub_cmd_vel    = self.create_publisher(Twist,  '/cmd_vel',    10)

        # Cache last steering command
        self._last_angular_z = 0.0

    @staticmethod
    def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
        return max(lo, min(hi, x))

    def cb_cmd_vel(self, msg: Twist):
        # Cache only the steering (angular.z) from teleop
        self._last_angular_z = msg.angular.z

    def cb_joy(self, msg: Joy):
        # Ensure 'raw' always defined for logging
        raw = 0.0

        # 1) Are we in brake-enabled mode?
        enabled = (
            len(msg.buttons) > self.enable_button
            and msg.buttons[self.enable_button] == 1
        )
        self.pub_mode.publish(Bool(data=enabled))

        # 2) Compute brake PWM duty if enabled
        if enabled and len(msg.axes) > self.trigger_axis:
            raw = self._clamp(msg.axes[self.trigger_axis])  # trigger axis value [+1..-1]
            normalized = (1.0 - raw) * 0.5                  # map to [0..1]
            duty = int(normalized * self.max_duty * self.duty_scale)
        else:
            duty = 0

        # 3) Publish brake PWM
        self.pub_pwm.publish(UInt8(data=duty))

        # 4) If braking (duty > 0), publish a Twist with linear.x=0 but keep last angular.z
        if duty > 0:
            twist = Twist()
            twist.linear.x  = 0.0
            twist.angular.z = self._last_angular_z
            self.get_logger().info('Braking: linear.x=0, angular.z kept at {:.2f}'.format(self._last_angular_z))
            self.pub_cmd_vel.publish(twist)
        # else: do nothing—teleop node’s /teleop_cmd_vel continues driving

        # 5) Debug
        self.get_logger().debug(f"enabled={enabled}, raw={raw:.2f}, duty={duty}")

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

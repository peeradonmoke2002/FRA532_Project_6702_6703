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
        self.enable_button = 6      # hold to enable braking input
        self.trigger_axis  = 5      # L2 trigger
        self.max_duty      = 255    # full-scale PWM
        self.duty_scale    = 1    # fraction of max_duty

        # === ROS interfaces ===
        self.create_subscription(Joy,  'joy',     self.cb_joy,  10)
        self.pub_mode       = self.create_publisher(Bool, 'mode',      10)
        self.pub_pwm        = self.create_publisher(UInt8,'pwm_duty', 10)
        self.pub_cmd_vel    = self.create_publisher(Twist,'cmd_vel',   10)

        # Pre-create zero-Twist for quick reuse
        self._zero_twist = Twist()

    @staticmethod
    def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
        return max(lo, min(hi, x))

    def cb_joy(self, msg: Joy):
        # 1) Publish mode (enabled or not)
        enabled = (
            len(msg.buttons) > self.enable_button
            and msg.buttons[self.enable_button] == 1
        )
        self.pub_mode.publish(Bool(data=enabled))

        # 2) Compute pwm_duty if enabled
        if enabled and len(msg.axes) > self.trigger_axis:
            raw = self._clamp(msg.axes[self.trigger_axis])  # [+1..-1]
            normalized = (1.0 - raw) * 0.5                  # [0..1]
            duty = int(normalized * self.max_duty * self.duty_scale)
        else:
            duty = 0

        # 3) Publish PWM
        self.pub_pwm.publish(UInt8(data=duty))

        # 4) If there is braking force (duty > 0), send zero cmd_vel
        if duty > 0:
            self.pub_cmd_vel.publish(self._zero_twist)

        # 5) Debug log
        self.get_logger().debug(f"enabled={enabled}, raw={raw if enabled else 'N/A'}, duty={duty}")

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

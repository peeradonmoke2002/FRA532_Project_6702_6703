#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')

        # ===== CONFIGURATION =====
        self.V_MAX           = 2.5       # m/s
        self.MAX_STEER_ANGLE = 0.698     # rad ≃ 40°
        self.WHEELBASE       = 0.263     # m
        self.W_MAX           = np.tan(self.MAX_STEER_ANGLE) / self.WHEELBASE

        # joystick mapping
        self.SPEED_AXIS          = 4   # right trigger
        self.STEER_AXIS          = 0   # left stick horizontal
        self.SPEED_ENABLE_BUTTON = 7   # dead-man switch
        self.DIR_TOGGLE_BUTTON   = 1   # forward/reverse toggle

        # state
        self.direction         = 1
        self._prev_dir_pressed = False

        # ===== ROS INTERFACES =====
        self.sub_joy = self.create_subscription(Joy,  'joy',    self.joy_cb, 10)
        self.pub_tw  = self.create_publisher(Twist, 'cmd_vel', 10)

        self.twist = Twist()

    def joy_cb(self, msg: Joy):
        # 1) Toggle forward/reverse on rising edge
        dir_pressed = bool(msg.buttons[self.DIR_TOGGLE_BUTTON])
        if dir_pressed and not self._prev_dir_pressed:
            self.direction *= -1
            self.get_logger().info(
                f'Direction → {"FORWARD" if self.direction > 0 else "REVERSE"}'
            )
        self._prev_dir_pressed = dir_pressed

        # 2) Compute speed (only while dead-man held)
        speed = 0.0
        enable = bool(msg.buttons[self.SPEED_ENABLE_BUTTON])
        if enable:
            raw = msg.axes[self.SPEED_AXIS]  # +1 unpressed → -1 full press
            norm = (1.0 - raw) * 0.5         # [0..1]
            speed = float(
                np.clip(norm * self.V_MAX * self.direction,
                        -self.V_MAX, self.V_MAX)
            )

        # 3) Steering only if enable button is pressed
        threshold = 0.1
        steer_angle = 0.0
        if enable:
            raw_st = msg.axes[self.STEER_AXIS]
            if raw_st < -threshold:
                steer_angle = -self.MAX_STEER_ANGLE  # Full Right
            elif raw_st > threshold:
                steer_angle = self.MAX_STEER_ANGLE   # Full Left
            else:
                steer_angle = 0.0                    # Center
        # else: steer_angle stays at 0.0 (Center)

        # Calculate angular velocity from steering angle (bicycle model)
        if steer_angle == 0.0:
            curvature = 0.0
        else:
            curvature = np.tan(steer_angle) / self.WHEELBASE
            curvature = float(np.clip(curvature, -self.W_MAX, self.W_MAX))

        # 4) Publish cmd_vel (linear & angular)
        self.twist.linear.x = speed
        self.twist.angular.z = curvature
        self.pub_tw.publish(self.twist)

        
def main(args=None):
    rclpy.init(args=args)
    node = JoyControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

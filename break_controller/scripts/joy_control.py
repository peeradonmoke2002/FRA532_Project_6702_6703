#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
import numpy as np

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')

        # ===== CONFIGURATION =====
        self.V_MAX           = 0.5       # m/s (initial)
        self.V_MAX_MIN       = 0.2       # m/s (lower bound)
        self.V_MAX_MAX       = 2.5       # m/s (upper bound)
        self.MAX_STEER_ANGLE = 0.698     # rad ≃ 40°
        self.WHEELBASE       = 0.263     # m
        self.W_MAX           = np.tan(self.MAX_STEER_ANGLE) / self.WHEELBASE

        # joystick mapping
        self.SPEED_AXIS          = 4   # right trigger
        self.STEER_AXIS          = 0   # left stick horizontal
        self.SPEED_ENABLE_BUTTON = 7   # dead-man switch
        self.DIR_TOGGLE_BUTTON   = 1   # forward/reverse toggle
        self.INC_SPEED_BUTTON    = 4   # Y button
        self.DEC_SPEED_BUTTON    = 0   # A button

        # state
        self.direction         = 1
        self._prev_dir_pressed = False
        self._prev_inc_pressed = False
        self._prev_dec_pressed = False
        
        self._break_pwm = 0   # store last break_pwm value


        # ===== ROS INTERFACES =====
        self.sub_joy = self.create_subscription(Joy,  '/joy',    self.joy_cb, 10)
        self.pub_tw  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_pwm = self.create_subscription(UInt8, '/break_pwm', self.pwm_cb,   10)


        self.twist = Twist()
        
    def pwm_cb(self, msg: UInt8):
            self._break_pwm = msg.data

    def joy_cb(self, msg: Joy):
        # 1) Toggle forward/reverse on rising edge
        dir_pressed = bool(msg.buttons[self.DIR_TOGGLE_BUTTON])
        if dir_pressed and not self._prev_dir_pressed:
            self.direction *= -1
            self.get_logger().info(
                f'Direction → {"FORWARD" if self.direction>0 else "REVERSE"}'
            )
        self._prev_dir_pressed = dir_pressed

        # --- Handle Speed Limit Adjustment (Y/A buttons) ---
        inc_pressed = bool(msg.buttons[self.INC_SPEED_BUTTON])
        dec_pressed = bool(msg.buttons[self.DEC_SPEED_BUTTON])
        if inc_pressed and not self._prev_inc_pressed:
            self.V_MAX = min(self.V_MAX * 1.1, self.V_MAX_MAX)
            self.get_logger().info(f'V_MAX increased: {self.V_MAX:.2f} m/s')
        if dec_pressed and not self._prev_dec_pressed:
            self.V_MAX = max(self.V_MAX * 0.9, self.V_MAX_MIN)
            self.get_logger().info(f'V_MAX decreased: {self.V_MAX:.2f} m/s')
        self._prev_inc_pressed = inc_pressed
        self._prev_dec_pressed = dec_pressed

        # --- Only allow speed and steering if ENABLE is held ---
        if msg.buttons[self.SPEED_ENABLE_BUTTON]:
            # Compute speed
            raw  = msg.axes[self.SPEED_AXIS]      # +1 unpressed → -1 full press
            norm = (1.0 - raw) * 0.5              # [0..1]
            speed = float(
                np.clip(norm * self.V_MAX * self.direction,
                        -self.V_MAX, self.V_MAX)
            )

            # Compute steering
            raw_st = msg.axes[self.STEER_AXIS]  # [-1..+1]
            threshold = 0.1
            if raw_st < -threshold:
                steer_angle = -self.MAX_STEER_ANGLE
            elif raw_st > threshold:
                steer_angle = self.MAX_STEER_ANGLE
            else:
                steer_angle = 0.0
        else:
            # Not enabled: no movement
            speed = 0.0
            steer_angle = 0.0

        # Calculate angular velocity from steering angle (bicycle model)
        if steer_angle == 0.0:
            curvature = 0.0
        else:
            curvature = np.tan(steer_angle) / self.WHEELBASE
            curvature = float(np.clip(curvature, -self.W_MAX, self.W_MAX))


        # # --- New: If braking, set linear.x = 0 ---
        # if self._break_pwm != 0:
        #     speed = 0.0

        # Publish cmd_vel (linear & angular)
        self.twist.linear.x  = speed
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

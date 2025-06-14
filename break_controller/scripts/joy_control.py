#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')

        # ===== CONFIGURATION =====
        self.V_MAX                = 2.5    # m/s
        self.W_MAX                = np.tan(0.698) / 0.263
        self.SPEED_AXIS           = 4      # right trigger
        self.STEER_AXIS           = 0      # left stick horizontal
        self.SPEED_ENABLE_BUTTON  = 7      # dead-man for speed
        self.DIR_TOGGLE_BUTTON    = 1      # flip fwd/rev

        # ===== STATE =====
        self.direction            = 1
        self._prev_dir_pressed    = False

        # ===== ROS INTERFACES =====
        self.sub   = self.create_subscription(Joy, 'joy',    self.joy_cb, 10)
        self.pub   = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

    def joy_cb(self, msg: Joy):
        # 1) Toggle forward/back on button 1 (rising edge)
        dir_pressed = (msg.buttons[self.DIR_TOGGLE_BUTTON] == 1)
        if dir_pressed and not self._prev_dir_pressed:
            self.direction *= -1
            mode = 'FORWARD' if self.direction>0 else 'REVERSE'
            self.get_logger().info(f'Direction → {mode}')
        self._prev_dir_pressed = dir_pressed

        # 2) Compute speed (only while button 7 held)
        speed = 0.0
        if msg.buttons[self.SPEED_ENABLE_BUTTON] == 1:
            # joy_node already applied deadzone & rate limiting
            raw = msg.axes[self.SPEED_AXIS]            # +1→unpressed, –1→full press
            normalized = (1.0 - raw) * 0.5             # maps [+1..–1]→[0..1]
            speed = normalized * self.V_MAX * self.direction

        # 3) Compute steering from axis 0 (±1 → ±W_MAX)
        steer_val = msg.axes[self.STEER_AXIS]
        if steer_val > 0.0:
            steer = +self.W_MAX   # full left
        elif steer_val < 0.0:
            steer = -self.W_MAX   # full right
        else:
            steer = 0.0           # centered

        # 4) Fill & publish
        self.twist.linear.x   = speed
        self.twist.angular.z  = steer
        self.pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

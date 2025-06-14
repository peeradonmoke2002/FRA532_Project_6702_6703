#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool, UInt8
from geometry_msgs.msg import Twist


class ABSBreakController(Node):
    def __init__(self):
        super().__init__('abs_break_controller')

        # === Configuration ===
        self.enable_button     = 6      # deadman button index
        self.trigger_axis      = 5      # L2 trigger axis index
        self.max_duty          = 255    # full-scale PWM
        self.duty_scale        = 0.5    # fraction of max_duty to use
        self.slip_threshold    = 0.20   # >20% slip triggers ABS
        self.release_duration  = 0.1    # seconds to release before re-apply
        self.abs_hysteresis    = 0.05   # extra margin to avoid chatter

        # Internal state
        self._cmd_duty       = 0
        self._wheel_speed    = 0.0
        self._veh_speed      = 0.0
        self._state          = 'NORMAL'  # or 'RELEASE'
        self._release_start  = None

        # Pre-create zero-Twist
        self._zero_twist = Twist()

        # === Publishers & Subscribers ===
        self.create_subscription(Joy,      'joy',      self.cb_joy,   10)
        self.create_subscription(Odometry,'odom',     self.cb_odom,  10)
        # wheel speed as Float32 on /wheel_speed
        self.create_subscription(Float32,  'wheel_speed', self.cb_wheel, 10)

        self.pub_mode     = self.create_publisher(Bool,  'mode',      10)
        self.pub_pwm      = self.create_publisher(UInt8, 'pwm_duty',  10)
        self.pub_cmd_vel  = self.create_publisher(Twist,  'cmd_vel',   10)

        # Timer for ABS control loop @ 50 Hz
        self.create_timer(1/50.0, self._abs_loop)

    def _clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def cb_joy(self, msg: Joy):
        # Deadman switch
        held = (len(msg.buttons) > self.enable_button
                and msg.buttons[self.enable_button] == 1)
        self.pub_mode.publish(Bool(data=held))

        # Compute raw duty if braking enabled
        if held and len(msg.axes) > self.trigger_axis:
            raw = self._clamp(msg.axes[self.trigger_axis], -1.0, 1.0)
            # normalize trigger: (1–raw)/2 in [0…1]
            norm = (1.0 - raw) * 0.5
            self._cmd_duty = int(norm * self.max_duty * self.duty_scale)
        else:
            self._cmd_duty = 0

    def cb_odom(self, msg: Odometry):
        # forward speed from odometry
        self._veh_speed = msg.twist.twist.linear.x

    def cb_wheel(self, msg: Float32):
        # wheel speed from sensor
        self._wheel_speed = msg.data

    def _abs_loop(self):
        """Runs at fixed rate to apply ABS logic and publish commands."""
        now = self.get_clock().now()

        if self._cmd_duty > 0:
            # compute current slip
            if self._veh_speed > 0.1:
                slip = (self._veh_speed - self._wheel_speed) / self._veh_speed
            else:
                slip = 0.0

            if self._state == 'NORMAL':
                if slip > self.slip_threshold:
                    # enter release state
                    self._state = 'RELEASE'
                    self._release_start = now
                    self.get_logger().warn(f"ABS release! slip={slip:.2f}")
            elif self._state == 'RELEASE':
                elapsed = (now - self._release_start).nanoseconds * 1e-9
                # wait for release_duration or until slip falls below threshold - hysteresis
                if elapsed >= self.release_duration or slip < (self.slip_threshold - self.abs_hysteresis):
                    self._state = 'NORMAL'
                    self.get_logger().info("ABS re-apply brake")

            # choose duty based on state
            duty = 0 if self._state == 'RELEASE' else self._cmd_duty
        else:
            # not braking at all
            duty = 0
            if self._state != 'NORMAL':
                self._state = 'NORMAL'

        # publish PWM and stop robot if braking
        self.pub_pwm.publish(UInt8(data=duty))
        if duty > 0:
            self.pub_cmd_vel.publish(self._zero_twist)

    def destroy_node(self):
        self.get_logger().info("Shutting down ABSBreakController")
        super().destroy_node()


def main():
    rclpy.init()
    node = ABSBreakController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

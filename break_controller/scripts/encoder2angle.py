#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float64

class Encoder2Angle(Node):
    def __init__(self):
        super().__init__('encoder2angle')
        # declare parameters with defaults
        self.declare_parameter('ppr', 2048)
        self.declare_parameter('offset', 0.0)

        # fetch parameters
        ppr    = self.get_parameter('ppr').value
        offset = self.get_parameter('offset').value
        self.counts_per_rev = float(ppr * 4)  # quadrature decoding

        self.get_logger().info(f'Using PPR={ppr}, offset={offset} counts')

        # publisher for computed steering angle (degrees)
        self.angle_pub = self.create_publisher(Float64, 'enc_steer', 10)

        # subscribe to raw encoder counts
        self.create_subscription(
            Int64,
            'enc_steer_raw',
            self.raw_callback,
            10
        )

    def raw_callback(self, msg: Int64):
        raw_counts = float(msg.data)
        # convert: (counts – offset) / counts_per_rev * 360°
        angle = ((raw_counts - self.get_parameter('offset').value)
                  / self.counts_per_rev) * 360.0

        out = Float64()
        out.data = angle
        self.angle_pub.publish(out)
        self.get_logger().debug(f'raw={raw_counts:.1f} → angle={angle:.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = Encoder2Angle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

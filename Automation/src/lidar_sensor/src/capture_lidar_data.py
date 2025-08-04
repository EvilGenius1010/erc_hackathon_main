import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',      # Change this if your topic name is different
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        # Print ranges (distance values for each beam)
        self.get_logger().info(f"Min range: {min(msg.ranges):.2f} m, Max range: {max(msg.ranges):.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = LidarListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

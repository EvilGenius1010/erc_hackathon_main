import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge


class SensorListener(Node):

    def __init__(self):
        super().__init__('sensor_listener')
        self.bridge = CvBridge()

        # Subscribe to standard camera
        self.subscription_camera = self.create_subscription(
            Image,
            '/my_camera/image_raw',
            self.camera_callback,
            10
        )

        # Subscribe to RGB-D camera (depth image)
        self.subscription_rgbd = self.create_subscription(
            Image,
            '/my_rgbd_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Subscribe to ultrasonic sensor
        self.subscription_ultrasonic = self.create_subscription(
            Range,
            '/ultrasonic',
            self.ultrasonic_callback,
            10
        )

    def camera_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f'Camera image received. Shape: {cv_image.shape}')

    def depth_callback(self, msg):
        # Depth image is typically 32FC1
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        mean_depth = np.nanmean(cv_depth)
        self.get_logger().info(f'Mean depth from RGB-D camera: {mean_depth:.2f} meters')

    def ultrasonic_callback(self, msg):
        self.get_logger().info(f"Ultrasonic sensor: range = {msg.range:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    sensor_listener = SensorListener()
    rclpy.spin(sensor_listener)
    sensor_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

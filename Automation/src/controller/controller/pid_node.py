# pid_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pid_controller import PIDController  # Import your PID class

class PIDVelocityNode(Node):
    def __init__(self):
        super().__init__('pid_velocity_node')
        # PID for linear velocity
        self.linear_pid = PIDController(kp=1.0, ki=0.0, kd=0.1, dt=0.1, output_limits=(0.0, 1.0))
        self.current_linear = 0.0
        # Subscribes to '/vel'
        self.create_subscription(Twist, '/vel', self.vel_callback, 10)
        # Replace this with your real feedback if available (odometry, sensors)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_linear = 0.0

    def vel_callback(self, msg):
        self.target_linear = msg.linear.x

    def timer_callback(self):
        # In real scenario, get self.current_linear from robot/odometry.
        pid_output = self.linear_pid.compute(self.target_linear, self.current_linear)
        twist = Twist()
        twist.linear.x = pid_output
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Target: {self.target_linear:.2f}, Current: {self.current_linear:.2f}, PID: {pid_output:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PIDVelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

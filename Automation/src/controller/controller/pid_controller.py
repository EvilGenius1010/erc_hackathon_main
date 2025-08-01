import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pid_controller import PIDController  # Import your PID class

class PIDControllerNode:
    def __init__(self,kp,kd,ki,dt=0.1,output_limits=(None,None)):
        self.kp=0
        self.kd=0
        self.ki=0
        self.output_limits=output_limits
        self.integral=0
        self.diff=0
        # self.error=0

        def compute(self, setpoint, measurement):
            error = setpoint - measurement
            self.integral += error * self.dt
            derivative = (error - self.prev_error) / self.dt
            output = (
                self.kp * error +
                self.ki * self.integral +
                self.kd * derivative
            )
            self.prev_error = error

            # Apply output limits
            lower, upper = self.output_limits
            if lower is not None:
                output = max(lower, output)
            if upper is not None:
                output = min(upper, output)
            return output

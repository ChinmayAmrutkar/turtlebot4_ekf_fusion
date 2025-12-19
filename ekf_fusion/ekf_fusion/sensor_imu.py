import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuNode(Node):
    def __init__(self):
        super().__init__('sensor_imu')
        
        # Subscribe to RAW data
        self.sub = self.create_subscription(Imu, '/imu', self.callback, 10)
        
        # Publish CLEAN data
        self.pub = self.create_publisher(Imu, '/imu_clean', 10)

        # Variables for Calibration
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0
        self.samples = []
        self.is_calibrated = False
        self.calibration_samples = 200  # Take 200 readings to calculate bias

        self.get_logger().info("Keep robot still! Calibrating IMU...")

    def callback(self, msg):
        # Step 1: Calibration Phase
        if not self.is_calibrated:
            self.samples.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            
            if len(self.samples) >= self.calibration_samples:
                arr = np.array(self.samples)
                self.bias_x = np.mean(arr[:,0])
                self.bias_y = np.mean(arr[:,1])
                self.bias_z = np.mean(arr[:,2])
                self.is_calibrated = True
                self.get_logger().info(f"IMU Calibrated. Bias Z: {self.bias_z:.5f}")
            return

        # Step 2: Processing Phase (Remove Bias)
        clean_msg = Imu()
        clean_msg.header = msg.header
        clean_msg.angular_velocity.x = msg.angular_velocity.x - self.bias_x
        clean_msg.angular_velocity.y = msg.angular_velocity.y - self.bias_y
        clean_msg.angular_velocity.z = msg.angular_velocity.z - self.bias_z
        
        # Pass through linear acceleration (gravity)
        clean_msg.linear_acceleration = msg.linear_acceleration
        
        self.pub.publish(clean_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
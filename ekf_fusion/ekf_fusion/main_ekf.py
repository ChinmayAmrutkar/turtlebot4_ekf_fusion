import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf_transformations
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MainEKF(Node):
    def __init__(self):
        super().__init__('main_ekf')

        # STATE [x, y, theta]
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1 # Uncertainty

        # MODEL NOISE (Q) & SENSOR NOISE (R)
        self.Q = np.diag([0.05, 0.05, 0.03])  
        self.R_lidar = np.diag([0.05, 0.05, 0.02])

        # INPUTS
        self.create_subscription(Imu, '/imu_clean', self.imu_cb, 10)
        self.create_subscription(Odometry, '/wheel_velocity', self.wheel_cb, 10)
        self.create_subscription(Odometry, '/lidar_odom', self.lidar_cb, 10)

        # OUTPUT
        self.pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.v = 0.0
        self.w = 0.0

    def predict(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        if dt > 1.0: return

        theta = self.x[2, 0]
        
        # Jacobian F
        F = np.eye(3)
        F[0, 2] = -self.v * math.sin(theta) * dt
        F[1, 2] =  self.v * math.cos(theta) * dt

        # State Prediction
        self.x[0] += self.v * math.cos(theta) * dt
        self.x[1] += self.v * math.sin(theta) * dt
        self.x[2] += self.w * dt
        
        # Covariance Prediction
        self.P = F @ self.P @ F.T + self.Q
        
        self.publish_state()

    def update(self, z, R):
        # H is Identity (we measure x,y,theta directly from lidar odom)
        y = z - self.x
        # Normalize Yaw Innovation
        y[2, 0] = math.atan2(math.sin(y[2, 0]), math.cos(y[2, 0]))
        
        S = self.P + R
        K = self.P @ np.linalg.inv(S)
        
        self.x = self.x + (K @ y)
        self.P = (np.eye(3) - K) @ self.P

    def imu_cb(self, msg):
        self.w = msg.angular_velocity.z
        self.predict() # High freq prediction

    def wheel_cb(self, msg):
        self.v = msg.twist.twist.linear.x

    def lidar_cb(self, msg):
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        z = np.array([[msg.pose.pose.position.x],
                      [msg.pose.pose.position.y],
                      [yaw]])
        self.update(z, self.R_lidar)

    def publish_state(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link_ekf"
        
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        q = tf_transformations.quaternion_from_euler(0, 0, float(self.x[2]))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        self.pub.publish(msg)

        # Broadcast TF
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = "base_link_ekf"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MainEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
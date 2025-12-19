import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf_transformations
import numpy as np
import math
from tf2_ros import TransformBroadcaster

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # --- 1. State Initialization ---
        # State Vector [x, y, theta]
        self.x = np.zeros((3, 1)) 
        
        # Covariance Matrix P (Initial uncertainty)
        self.P = np.eye(3) * 0.1 

        # --- 2. Tuning Parameters (The "Art" of Kalman Filtering) ---
        # Process Noise Q (Uncertainty in our Prediction/Model)
        # Low = Trust Model, High = Trust Sensors
        self.Q = np.diag([0.05, 0.05, 0.03]) 

        # Measurement Noise R (Uncertainty in Sensors)
        # R_odom: Wheel odometry is smooth but drifts
        self.R_odom = np.diag([0.1, 0.1, 0.05])
        # R_lidar: Scan matching is accurate but can jump
        self.R_lidar = np.diag([0.05, 0.05, 0.02])
        # R_visual: Visual odometry is good but sensitive to light
        self.R_visual = np.diag([0.08, 0.08, 0.04])

        # --- 3. Subscribers ---
        # Prediction Input: IMU (Angular Vel) + Wheel (Linear Vel)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Correction Inputs:
        # Note: Lidar/Camera need to publish Odometry or Pose messages.
        self.create_subscription(Odometry, '/lidar_odom', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/visual_odom', self.visual_callback, 10)

        # --- 4. Publishers ---
        self.ekf_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal Variables
        self.last_time = self.get_clock().now()
        self.current_v = 0.0  # Linear velocity from Wheel Odom
        self.current_w = 0.0  # Angular velocity from IMU

    # ==========================================================
    # PREDICTION STEP (Process Model)
    # ==========================================================
    def predict(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 1.0: # Ignore large time jumps
            return

        theta = self.x[2, 0]
        v = self.current_v
        w = self.current_w

        # Jacobians (Linearization of the motion model)
        # Motion Model:
        # x_new = x + v * cos(theta) * dt
        # y_new = y + v * sin(theta) * dt
        # theta_new = theta + w * dt

        # F Matrix (State Transition Jacobian)
        F = np.eye(3)
        F[0, 2] = -v * math.sin(theta) * dt
        F[1, 2] =  v * math.cos(theta) * dt

        # Predict State
        self.x[0, 0] = self.x[0, 0] + v * math.cos(theta) * dt
        self.x[1, 0] = self.x[1, 0] + v * math.sin(theta) * dt
        self.x[2, 0] = self.x[2, 0] + w * dt

        # Normalize Yaw to [-pi, pi]
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

        # Predict Covariance
        self.P = F @ self.P @ F.T + self.Q

        # Publish the prediction immediately (high freq)
        self.publish_ekf()

    # ==========================================================
    # CORRECTION STEP (Measurement Update)
    # ==========================================================
    def update(self, z, R_source):
        # z: Measurement Vector [x, y, theta]
        # R_source: Measurement Noise Matrix for this specific sensor

        # H Matrix (Measurement Jacobian) - Identity because we measure state directly
        H = np.eye(3)

        # Innovation (Residual) y = z - Hx
        y = z - (H @ self.x)

        # Normalize Yaw Innovation (Crucial!)
        y[2, 0] = math.atan2(math.sin(y[2, 0]), math.cos(y[2, 0]))

        # Innovation Covariance S = H P H.T + R
        S = H @ self.P @ H.T + R_source

        # Kalman Gain K = P H.T S^-1
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # If matrix singular, skip update
            return

        # Update State x = x + K y
        self.x = self.x + (K @ y)

        # Normalize Yaw Again
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

        # Update Covariance P = (I - K H) P
        I = np.eye(3)
        self.P = (I - (K @ H)) @ self.P

    # ==========================================================
    # CALLBACKS
    # ==========================================================
    def imu_callback(self, msg):
        # We use IMU Gyro for PREDICTION (Angular Velocity)
        self.current_w = msg.angular_velocity.z
        # Trigger prediction every time we get an IMU msg (usually high freq)
        self.predict()

    def odom_callback(self, msg):
        # We use Wheel Odom Linear Velocity for PREDICTION
        self.current_v = msg.twist.twist.linear.x
        
        # OPTIONAL: Use Wheel Position for Correction (Low weight to prevent drift)
        # Extract Pose
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        z = np.array([[msg.pose.pose.position.x], 
                      [msg.pose.pose.position.y], 
                      [yaw]])
        self.update(z, self.R_odom)

    def lidar_callback(self, msg):
        # Lidar Matcher provides absolute pose
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        z = np.array([[msg.pose.pose.position.x], 
                      [msg.pose.pose.position.y], 
                      [yaw]])
        self.update(z, self.R_lidar)

    def visual_callback(self, msg):
        # Visual Odom provides absolute pose
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        z = np.array([[msg.pose.pose.position.x], 
                      [msg.pose.pose.position.y], 
                      [yaw]])
        self.update(z, self.R_visual)

    # ==========================================================
    # UTILS
    # ==========================================================
    def publish_ekf(self):
        # Publish Odometry Message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link_ekf"

        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])
        
        q = tf_transformations.quaternion_from_euler(0, 0, float(self.x[2, 0]))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Publish Covariance (Diagonal only for viz)
        msg.pose.covariance[0] = self.P[0,0]
        msg.pose.covariance[7] = self.P[1,1]
        msg.pose.covariance[35] = self.P[2,2]

        self.ekf_pub.publish(msg)

        # Broadcast Transform (odom -> base_link_ekf)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link_ekf"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
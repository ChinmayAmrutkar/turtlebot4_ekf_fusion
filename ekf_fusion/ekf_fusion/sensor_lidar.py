import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
import tf_transformations
import math

class LidarOdomNode(Node):
    def __init__(self):
        super().__init__('sensor_lidar')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Odometry, '/lidar_odom', 10)

        self.prev_pcd = None
        self.global_transform = np.eye(4) # Start at 0,0,0
        
        # Tuning: Max distance to match points (0.5 meters)
        self.icp_threshold = 0.5 

    def scan_to_pointcloud(self, msg):
        # Convert ROS LaserScan to Open3D PointCloud
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Filter infinite/bad ranges
        valid_mask = np.isfinite(ranges) & (ranges > 0.1) & (ranges < msg.range_max)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        # Polar to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        points = np.stack([x, y, z], axis=1)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Downsample for speed (Voxel Grid Filter)
        pcd = pcd.voxel_down_sample(voxel_size=0.05)
        return pcd

    def scan_callback(self, msg):
        current_pcd = self.scan_to_pointcloud(msg)

        if self.prev_pcd is None:
            self.prev_pcd = current_pcd
            return

        # Run ICP (Point-to-Plane is faster, but Point-to-Point is safer for 2D)
        # Using Point-to-Point registration
        reg_p2p = o3d.pipelines.registration.registration_icp(
            current_pcd, self.prev_pcd, self.icp_threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        # Update Global Transform
        # The result 'reg_p2p.transformation' is the move from Current -> Prev
        # We want to accumulate this movement.
        delta_transform = reg_p2p.transformation
        self.global_transform = self.global_transform @ delta_transform

        # Extract X, Y, Yaw from the global transform
        x = self.global_transform[0, 3]
        y = self.global_transform[1, 3]
        yaw = math.atan2(self.global_transform[1, 0], self.global_transform[0, 0])

        # Publish
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "odom"
        odom.child_frame_id = "lidar_link"
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Hardcoded High Precision Covariance (We trust Lidar)
        odom.pose.covariance[0] = 0.01 # X
        odom.pose.covariance[7] = 0.01 # Y
        odom.pose.covariance[35] = 0.01 # Yaw

        self.pub.publish(odom)
        
        # Update Previous Frame
        self.prev_pcd = current_pcd

def main(args=None):
    rclpy.init(args=args)
    node = LidarOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
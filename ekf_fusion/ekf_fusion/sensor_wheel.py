import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class WheelNode(Node):
    def __init__(self):
        super().__init__('sensor_wheel')
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/wheel_velocity', 10)

    def callback(self, msg):
        # We only care about Twist (Velocity), not Pose (Position)
        # We republish just to ensure timestamp sync if needed later
        output = Odometry()
        output.header = msg.header
        output.child_frame_id = "base_link"
        
        # Pass purely the velocity
        output.twist = msg.twist
        
        # Set Covariance (Confidence)
        # We are very confident in Linear Velocity from wheels
        output.twist.covariance = msg.twist.covariance
        
        self.pub.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = WheelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
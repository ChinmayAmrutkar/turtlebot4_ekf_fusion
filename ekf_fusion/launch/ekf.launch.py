from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ekf_fusion', executable='sensor_imu', name='sensor_imu'),
        Node(package='ekf_fusion', executable='sensor_wheel', name='sensor_wheel'),
        Node(package='ekf_fusion', executable='sensor_lidar', name='sensor_lidar'),
        Node(package='ekf_fusion', executable='main_ekf', name='main_ekf'),
    ])
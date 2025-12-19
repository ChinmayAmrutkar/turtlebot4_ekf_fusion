# Turtlebot 4 EKF Sensor Fusion

This project implements a custom **Modular Extended Kalman Filter (EKF)** for the Turtlebot 4 running ROS 2 Humble.

Unlike standard packages, this uses a micro-services architecture where each sensor has its own dedicated processing script, fusing data into a robust global estimate.

## Architecture

The system is split into 4 modular nodes:

### 1. Sensor Processing (Pre-Processing)
* **`sensor_imu.py`:** Calibrates the IMU at startup to remove gyro bias and publishes clean angular velocity.
* **`sensor_wheel.py`:** Extracts pure linear velocity from wheel encoders.
* **`sensor_lidar.py`:** Performs **Scan Matching (ICP)** using Open3D to calculate absolute robot motion from raw laser scans (replaces `rf2o`).

### 2. Fusion Engine (`main_ekf.py`)
* **Prediction:** Uses Wheel Velocity ($v$) + Clean Gyro ($\omega$).
* **Correction:** Corrects drift using the Lidar Odometry calculated by `sensor_lidar`.

---

## Prerequisites

### 1. System Requirements
* **Robot:** Turtlebot 4 (Raspberry Pi 4 + Create 3 Base)
* **OS:** Ubuntu 22.04 (Jammy)
* **ROS Distro:** ROS 2 Humble
* **Network:** CycloneDDS configured properly.

### 2. Dependencies
We removed external ROS packages in favor of standard Python libraries. You need these installed on your Host PC / Robot:
```bash
# Python Math & Computer Vision Libraries
pip3 install numpy transforms3d open3d opencv-python scipy

# ROS 2 TF Transformations
sudo apt install ros-humble-tf-transformations
```
---

## Installation

1.  **Create a Workspace**:
```bash
mkdir -p ~/ekf_ws/src
cd ~/ekf_ws/src
```
2.  **Clone the Repository**:
```bash
git clone https://github.com/ChinmayAmrutkar/turtlebot4_ekf_fusion.git
```
3.  **Install ROS Dependencies**:
```bash
cd ~/ekf_ws
rosdep install --from-paths src --ignore-src -r -y
```
4.  **Build**:
```bash
colcon build --symlink-install
source install/setup.bash
```
---

## Usage

### Step 1: Launch the Robot
Ensure the Turtlebot 4 is turned on and active.
```bash
ros2 topic list
```

### Step 2: Launch the EKF Stack
We have a master launch file that starts all 4 nodes (`sensor_imu`, `sensor_wheel`, `sensor_lidar`, `main_ekf`) automatically.
```bash
ros2 launch ekf_fusion ekf.launch.py
```
*Note: Keep the robot still for the first 5 seconds while `sensor_imu` calibrates!*

### Step 3: Visualize
Open RViz2:
```bash
rviz2
```
* Set **Fixed Frame** to `odom`.
* Add **Odometry** display -> Topic `/ekf_odom`.
* Add **Odometry** display -> Topic `/lidar_odom` (to see the raw ICP result).

---

## Configuration & Tuning

### Tuning the EKF (`main_ekf.py`)
Adjust the Kalman matrices to trust specific sensors:
* **Process Noise (`self.Q`):** Trust the prediction (Wheels/IMU) less? Increase this.
* **Measurement Noise (`self.R`):** Trust the Lidar ICP less? Increase `self.R_lidar`.

### Tuning the Lidar ICP (`sensor_lidar.py`)
* **`self.icp_threshold`:** Currently set to **0.5m**. If the robot moves faster than this between frames, ICP might fail. Increase if the robot is fast, decrease if scans are noisy.

---

## Troubleshooting

**1. "IMU Calibration stuck"**
* Ensure the robot is completely stationary when you launch the code. It takes 200 samples (~2 seconds) to calculate bias.

**2. "Module 'open3d' not found"**
* You missed the pip install step. Run `pip3 install open3d`.

**3. "Matrix Singular" Error**
* This usually means the Covariance Matrix collapsed. Restart the launch file.

**4. Robot Drifting in RViz**
* Check if `sensor_lidar` is publishing `/lidar_odom`. If the environment is featureless (long empty hallway), ICP cannot lock on.

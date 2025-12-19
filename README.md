# Turtlebot 4 EKF Sensor Fusion

This project implements a custom **Extended Kalman Filter (EKF)** for the Turtlebot 4 running ROS 2 Humble. 

It fuses data from multiple sensors to create a robust, drift-free estimate of the robot's position and heading (`/ekf_odom`).

## How It Works (The Algorithm)

This EKF uses a **Kinematic Motion Model** to split the task into Prediction and Correction steps.

### 1. Prediction Step (High Frequency)
We predict the robot's next state based on its control inputs.
* **Source:** * **Linear Velocity ($v$):** From Wheel Encoders (`/odom`).
    * **Angular Velocity ($\omega$):** From IMU Gyroscope (`/imu`).
* **Why?** Wheel encoders are good for speed but drift in rotation. The IMU is excellent for detecting rotation changes. This combination gives the smoothest prediction.

### 2. Correction Step (Low Frequency)
We correct the predicted position using absolute measurements from external sources.
* **Wheel Odometry (`/odom`):** Used with low confidence to anchor the scale.
* **Lidar Odometry (`/lidar_odom`):** Derived from Scan Matching (via `rf2o`). Corrects drift using the map environment.
* **Visual Odometry (`/visual_odom`):** From OAK-D camera. Corrects drift using visual features.

---

## Prerequisites

### 1. System Requirements
* **Robot:** Turtlebot 4 (Raspberry Pi 4 + Create 3 Base)
* **OS:** Ubuntu 22.04 (Jammy)
* **ROS Distro:** ROS 2 Humble
* **Network:** CycloneDDS configured properly (See `cyclonedds_pc.xml` setup).

### 2. Dependencies
You need these Python libraries and ROS packages installed on your Host PC / Robot:

```bash
# Python Math Libraries
pip3 install numpy transforms3d

# ROS 2 TF Transformations
sudo apt install ros-humble-tf-transformations

# Lidar Odometry Generator (Required for Lidar fusion)
sudo apt install ros-humble-rf2o-laser-odometry
```
# Turtlebot 4 EKF Sensor Fusion

This project implements a custom **Extended Kalman Filter (EKF)** for the Turtlebot 4 running ROS 2 Humble.

It fuses data from multiple sensors to create a robust, drift-free estimate of the robot's position and heading (`/ekf_odom`).

## How It Works (The Algorithm)

This EKF uses a **Kinematic Motion Model** to split the task into Prediction and Correction steps.

### 1. Prediction Step (High Frequency)
We predict the robot's next state based on its control inputs.
* **Source:** * **Linear Velocity (v):** From Wheel Encoders (`/odom`).
    * **Angular Velocity (w):** From IMU Gyroscope (`/imu`).
* **Why?** Wheel encoders are good for speed but drift in rotation. The IMU is excellent for detecting rotation changes. This combination gives the smoothest prediction.

### 2. Correction Step (Low Frequency)
We correct the predicted position using absolute measurements from external sources.
* **Wheel Odometry (`/odom`):** Used with low confidence to anchor the scale.
* **Lidar Odometry (`/lidar_odom`):** Derived from Scan Matching (via `rf2o`). Corrects drift using the map environment.
* **Visual Odometry (`/visual_odom`):** From OAK-D camera. Corrects drift using visual features.

---

## Prerequisites

### 1. System Requirements
* **Robot:** Turtlebot 4 (Raspberry Pi 4 + Create 3 Base)
* **OS:** Ubuntu 22.04 (Jammy)
* **ROS Distro:** ROS 2 Humble
* **Network:** CycloneDDS configured properly (See `cyclonedds_pc.xml` setup).

### 2. Dependencies
You need these Python libraries and ROS packages installed on your Host PC / Robot:
```bash
# Python Math Libraries
pip3 install numpy transforms3d
```
```bash
# ROS 2 TF Transformations
sudo apt install ros-humble-tf-transformations
```
```bash
# Lidar Odometry Generator (Required for Lidar fusion)
sudo apt install ros-humble-rf2o-laser-odometry
```
---

## Installation

1.  **Create a Workspace** (if you haven't already):
```bash
mkdir -p ~/ekf_ws/src
cd ~/ekf_ws/src
```

2.  **Clone the Repository:**
```bash
git clone https://github.com/ChinmayAmrutkar/turtlebot4_ekf_fusion.git
```

3.  **Install ROS Dependencies:**
```bash
cd ~/ekf_ws
rosdep install --from-paths src --ignore-src -r -y
```

4.  **Build:**
```bash
colcon build --symlink-install
source install/setup.bash
```
---

## Usage

### Step 1: Launch the Robot
Ensure the Turtlebot 4 is turned on and you can see topics on your host PC.
```bash
ros2 topic list
```
### Step 2: Start Lidar Odometry (Scan Matcher)
The EKF needs an Odometry message, not raw laser scans. We use `rf2o` to convert the scan data.
```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py laser_scan_topic:=/scan odom_topic:=/lidar_odom
```

### Step 3: Run the EKF Node
```bash
ros2 run ekf_fusion ekf_node
```

### Step 4: Visualize
Open RViz2:
```bash
rviz2
```

* Set **Fixed Frame** to `odom`.
* Add **Odometry** display -> Topic `/ekf_odom`.
* (Optional) Compare it by adding another Odometry display for `/odom`.

---

## Configuration & Tuning

You can tune the Kalman Filter matrices inside `ekf_node.py` to trust certain sensors more or less.

* **Process Noise (`self.Q`):** Increase this if the robot moves unpredictably (slip).
* **Measurement Noise (`self.R`):**
    * **`R_odom`:** Trust wheel encoders less? Increase these values.
    * **`R_lidar`:** Trust Lidar more? Decrease these values.

    # Example: Trust Lidar Very Highly
    self.R_lidar = np.diag([0.01, 0.01, 0.01]) 

---

## Troubleshooting

**1. "Topic /scan not found"**
* Ensure the robot is not docked (power saving disables Lidar).
* Check CycloneDDS configuration.

**2. "Matrix Singular" Error**
* This happens if the covariance matrix collapses. Restart the node. It usually means your `R` values are too small (too confident).

**3. Robot spinning in RViz**
* Check if `ROS_DOMAIN_ID` matches on both Robot and PC.
* Ensure IMU calibration on the Create 3 base is correct.

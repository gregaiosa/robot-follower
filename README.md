# ROS 2 Autonomous Human Following Robot

[![Portfolio](https://img.shields.io/badge/View_Portfolio_Post-blue?style=for-the-badge)](https://gregaiosa.github.io/projects/human_following_robot/)

This repository contains the ROS 2 workspace for an autonomous human-following system implemented on a Clearpath Jackal UGV. The system leverages YOLO object/pose detection, the Nav2 stack, and SLAM Toolbox to dynamically track, pursue, and navigate toward a designated person in real-time without the use of wearable sensors.

## System Overview

* **Perception:** Uses YOLO26 (specifically `yolo26n-pose.pt` and `person.pt`) to detect human targets and extract joint keypoints from a RealSense D435i RGB-D stream.
* **Gesture Control:** Maps extracted hand/arm keypoints to specific control commands, allowing the user to initiate or halt following behavior visually.
* **Navigation:** Integrates Nav2 and SLAM Toolbox with a Velodyne LiDAR for dynamic obstacle avoidance. 
* **Reacquisition:** If line-of-sight is broken, a state machine calculates the target's last known position and utilizes quaternion math to align the robot's goal for exploration and reacquisition.

## Hardware Requirements
* Clearpath Jackal UGV
* RealSense D435i Camera
* Velodyne LiDAR (VLP-16 or similar)
* Compute Node (Tested on 4th-Gen Intel i5)

## Software Dependencies
* ROS 2
* `nav2` / `navigation2` stack
* `slam_toolbox`
* `ultralytics` (for YOLO26)
* `realsense2_camera`

## Installation

1. Clone this repository into the `src` folder of your ROS 2 workspace:
   ```
   cd ~/your_ws/src
   git clone [https://github.com/gregaiosa/robot-follower.git](https://github.com/gregaiosa/robot-follower.git)
   ```
2. Install dependencies using `rosdep`:
   ```
   cd ~/your_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the workspace:
   ```
   colcon build --packages-select robot_follower
   source install/setup.bash
   ```

## Usage

The system is split into distinct launch files to allow for mapping new environments prior to deploying the vision tracking nodes.

### 1. Mapping the Environment
To create a new map of an unknown environment using SLAM Toolbox and the Velodyne LiDAR:
```
ros2 launch robot_follower mapping.launch.xml
```

### 2. Autonomous Navigation (Localization)
Once a map is saved, launch the Nav2 stack and localization nodes to navigate within the known environment:
```
ros2 launch robot_follower local.launch.xml map:=<path_to_your_map.yaml>
```

### 3. Vision Tracking & Control
With the navigation stack active, launch the perception and control nodes. You must specify the YOLO model you wish to use (models are located in the `robot_follower/models/` directory, e.g., `best.pt`, `person.pt`, or `yolo26n-pose.pt`):
```
ros2 launch robot_follower vision.launch.xml model:=<model_name.pt>
```

## Node Architecture

### `vision.py` (Perception & Spatial Mapping)
* **Inference Switching:** Dynamically switches between YOLO object detection and YOLO pose detection states depending on the model string provided at launch.
* **Spatial Calculation:** Subscribes to `/image_raw/compressed` and depth topics to extract bounding boxes or joint keypoints. It maps 2D pixel coordinates to 3D space using the camera's intrinsic matrix (`fx`, `fy`, `cx`, `cy`).
* **Signal Smoothing:** Applies an Exponential Moving Average (EMA) filter to the raw `[x, y, z]` coordinates to stabilize the target's estimated pose.
* **Outputs:** Broadcasts a `tf2` transform from `camera_0_depth_optical_frame` to a new `person` frame and publishes the data as a `PoseStamped` message.

### `control.py` (Navigation & State Management)
* **Coordinate Transformation:** Uses `tf2_geometry_msgs` to transform the incoming `person` pose into the `odom` frame for absolute tracking, and the `base_link` frame to determine relative left/right orientation.
* **Pursuit Logic:** Calculates a target goal that maintains a strict 1.0-meter follow distance from the person.
* **Deadband Filtering:** Implements a 0.25-meter deadband radius; the node will ignore minor target jitters to prevent spamming the Nav2 MPPI controller with micro-updates.
* **Watchdog & Reacquisition:** Runs a 0.5-second timer to monitor target visibility. If the target is lost for greater than 2.0 seconds, it cancels the active `MapsToPose` goal. It then dispatches a `Spin` action server goal (a full 6.28 radian sweep) in the rotational direction (CW or CCW) where the person was last detected.

### `led_control.py` (Hardware Feedback)
* Manages state indication via a serial connection (`/dev/ttyUSB0` at 9600 baud) to a custom LED array.
* **States:** * **Green:** Target successfully acquired and tracked.
  * **Yellow:** Target recognized in the RGB frame, but depth data is invalid (e.g., target is out of depth sensor range).
  * **Red:** Target lost; no bounding box or keypoints detected.

## Limitations & Future Work
* **Compute Bottleneck:** Currently tested on a 4th-generation i5 CPU. Inference speed and tracking frequency are bottlenecked by CPU processing. Future iterations should offload YOLO inference to a dedicated GPU (e.g., Jetson Orin) to allow for more aggressive dynamic tracking.
* **Occlusion Handling:** Rapid occlusions in crowded environments can occasionally confuse the tracker. Implementing a Kalman filter for trajectory prediction could improve reacquisition speed.
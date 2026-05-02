# AI for Industry Challenge - Autonomous Cable Insertion

This repository contains our implementation for the **AI for Industry Challenge**, a robotics competition focused on solving complex manufacturing tasks. The goal is to enable a robotic arm to autonomously perform high-precision cable insertion into varying network port configurations.

## Project Summary

The challenge involves manipulating a UR5e robotic arm in a Gazebo simulation to insert SFP and SC cables into a task board. Our solution focuses on building a robust, vision-based perception and control pipeline that can adapt to randomized environment layouts.

---

## 🛠️ Technical Implementation: `aic_model`

We implemented the core orchestration layer for the robotic policy within the `aic_model` package, focusing on reliability and modularity.

### Core Architecture: ROS 2 Lifecycle Management
The system is built as a **ROS 2 Lifecycle Node**, ensuring deterministic state transitions (Configure, Activate, Deactivate) and high reliability during long-running autonomous trials.

### Key Features
- **Unified Robot Interface:** Created a clean abstraction for robot motion that seamlessly handles transitions between Cartesian and Joint-space control modes.
- **Vision & Pose Estimation:** Developed a perception pipeline utilizing YOLO-Pose for real-time keypoint detection and PnP algorithms to accurately estimate the 6D pose of cables and ports.
- **Sensor Fusion:** Integrated real-time synchronization of RGB camera streams with force-torque (F/T) sensor data to enable future work on hybrid vision/force-compliant insertion.

---

## 🧬 Synthetic Data Generation 

To solve the perception problem without manual labeling, we built a sophisticated **Synthetic Data Generation** system in `GenerateData.py` that automates the creation of training datasets for YOLO-Pose.

### Automated Labeling & Projection
- **Ground-Truth Projection:** Instead of manual annotation, the engine leverages the simulation's internal transform tree (TF).
- **Computer Vision Integration:** It projects 3D port coordinates into the 2D image plane using camera intrinsic matrices, automatically generating YOLO-compatible labels (bounding boxes and keypoints) for every visible port.
- **Multi-Port Support:** The system dynamically identifies all active ports on the board, handling various NIC cards and rail configurations in a single frame.

### Environment Randomization (Domain Randomization)
To ensure the model generalizes to real-world variability, we implemented an automated "Domain Randomization" loop:
- **Dynamic Xacro Expansion:** The engine uses ROS services to periodically delete the environment and spawn a completely fresh task board with a randomized layout.
- **Pose & Angle Variability:** Every 25 frames, the system randomizes the board's position, yaw, and the robot's viewing angle, ensuring the dataset captures a wide range of perspectives and occlusion patterns.

---

## 🚀 Getting Started

### Data Generation
Run the automated engine to generate a synthetic dataset:
```bash
# Start sim with randomization services
pixi run ros2 launch aic_training_utils aic_training_gz_bringup.launch.py spawn_task_board:=true spawn_cable:=true ground_truth:=true

# Execute data generation policy
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=my_policy_node.GenerateData
```

### Policy Execution
Run the core insertion policy:
```bash
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=my_policy_node.InsertCable
```

## 🛠️ Tech Stack
- **Framework:** ROS 2 (Kilted Kaiju)
- **Simulation:** Gazebo (Sim), Xacro
- **Languages:** Python, C++
- **Perception Goal:** YOLO-Pose (Keypoint Detection)
- **Control:** Cartesian/Joint-space Position & Velocity Control

---

## 📄 License
This project is licensed under the Apache License 2.0. The [aic_isaac](./aic_utils/aic_isaac/) folder contains files licensed under BSD-3.

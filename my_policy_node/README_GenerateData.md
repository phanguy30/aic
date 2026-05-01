# YOLO-Pose Data Generation Walkthrough

This document explains how the `GenerateData.py` policy node works and how to run it to generate training data for YOLO-Pose.

## What it Accomplishes

1. **Multi-Port YOLO Labeling:** 
   - The TF processing loop in `GenerateData.py` dynamically iterates through all 5 possible NIC card slots and both SC/SFP rails. 
   - It checks every single port to see if it exists on the current board and if it's visible within the camera's frame. 
   - It correctly generates a multi-line YOLO `.txt` label file, labeling every single port in the image as `class_id = 0`.

2. **Automated Scene Randomization:**
   - It uses Python service clients for `/expand_xacro`, `/gz_server/delete_entity`, and `/gz_server/spawn_entity`.
   - Every 25 images, the robot arm briefly pauses, the physics engine deletes the old task board, randomly decides which rails and modules to spawn (and at what angle), generates the new physics XML, and drops a completely fresh, randomized task board into the scene!

## How to Test and Run

1. Make sure you are in the `pixi` shell and rebuild your packages:
   ```bash
   pixi shell
   pixi reinstall ros-kilted-my-policy-node
   ```
2. Start the simulation using the **training bringup** (this exposes the `expand_xacro` service required for the randomization feature):
   ```bash
   pixi run ros2 launch aic_training_utils aic_training_gz_bringup.launch.py spawn_task_board:=true spawn_cable:=true ground_truth:=true
   ```
3. In a new terminal, run the data generation policy:
   ```bash
   pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=my_policy_node.GenerateData
   ```

You will see the robot hovering above the board, gathering data into the `yolo_dataset` folder. Every 25 images, the board will vanish and instantly be replaced with a totally different layout!

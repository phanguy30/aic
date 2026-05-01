# My Policy Node

This package contains the custom policy implementation for the AI for Industry Challenge. The policy handles receiving observations from the robot's cameras and sensors, and commands the robot to perform the cable insertion task.

## Architecture

Our approach is divided into 3 main phases within the `insert_cable` loop (found in `my_policy_node/WaveArm.py`):

### Phase 1: Observation Gathering & Vision (WIP)
Instead of relying on ground truth transforms (like the CheatCode policy), this policy relies on actual visual input.
- **Current State:** A placeholder method `estimate_port_pose_from_image()` is set up.
- **Next Steps for the Team:** We need to implement computer vision logic (e.g., OpenCV template matching, color tracking, or a DL model) inside this method to calculate the target `Pose` of the port based on the `observation.center_image`.

### Phase 2: Approach Trajectory
Using the estimated coordinates from Phase 1, the policy commands the arm to move to a `hover_pose` exactly 20cm above the target. It waits briefly to allow the arm to stabilize.

### Phase 3: Insertion & Force-Compliance
The robot slowly descends along the Z-axis (lowering by 5mm at a time). 
To prevent hardware damage and adapt to slight visual misalignments, we monitor the `wrist_wrench` sensor in real-time.
- If the Z-axis force exceeds a safe threshold (e.g., 15 Newtons), it means the plug has hit the plastic casing. 
- The robot will back off slightly. 
- **Next Steps for the Team:** We can implement a "spiral search" pattern here to slightly oscillate the X/Y coordinates until the plug "feels" its way into the hole.

## Getting Started

Make sure you are in the `pixi` environment. If you make any changes to the python files, you must reinstall the package before running.

```bash
# Enter the environment
pixi shell

# Reinstall the package
pixi reinstall ros-kilted-my-policy-node

# Run the policy
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=my_policy_node.WaveArm
```

## Potential Workflows

Depending on the team's decision, we can take two different paths to solve this challenge.

### Workflow A: The Modular Approach (YOLO-Pose)
This approach uses a keypoint detection model (YOLO-Pose) to find the port in the image, and classical math to calculate its 3D depth. This is highly interpretable, lightning fast, and very easy to debug.

1. **Data Generation**: 
   Write a ROS 2 node that subscribes to the camera images and the ground-truth `tf` tree. Save `.jpg` images of the task board, and calculate where the 4 corners of the port appear in the 2D image. Save these 4 (x,y) keypoints as your training labels.
2. **Model Training (YOLO-Pose)**: 
   Train a YOLO-Pose model on your generated dataset. The model will learn to look at a raw camera image and predict the 2D pixel locations of the 4 port corners.
3. **Integration & 3D Math**: 
   - Load your trained YOLO-Pose model inside `estimate_port_pose_from_image()`.
   - Pass `observation.center_image` to the model to get the 4 corner pixels.
   - Use OpenCV's `cv2.solvePnP()` function. Feed it your 4 predicted 2D pixels, the known 3D physical dimensions of the port, and the `camera_info` matrix. OpenCV will instantly return the exact 6D Pose (X, Y, Z, Roll, Pitch, Yaw) of the port!
   - Pass that pose to our `CheatCode.py` trajectory logic to physically insert the cable.

### Workflow B: The End-to-End Approach (LeRobot)
This approach ignores manual geometry and tries to map raw pixels directly to robot movements using Imitation Learning (like ACT).

1. **Data Generation**: 
   You must manually teleoperate the robot using a keyboard or SpaceMouse to solve the task repeatedly. The more demonstrations you collect, the better the model will perform.
   > **Important:** Before running these commands, log in to Hugging Face by running `huggingface-cli login` in your terminal! Replace `YOUR_HF_USERNAME` with your actual username below.

   ```bash
   # Make sure you are inside the repo
   pixi run lerobot-record \
     --robot.type=aic_controller --robot.id=aic \
     --teleop.type=aic_keyboard_ee --teleop.id=aic \
     --robot.teleop_target_mode=cartesian --robot.teleop_frame_id=base_link \
     --dataset.repo_id=YOUR_HF_USERNAME/aic_cable_dataset \
     --dataset.single_task="insert the cable" \
     --dataset.push_to_hub=true \
     --display_data=true
   ```

2. **Model Training (Fine-Tuning)**: 
   Once you have enough data recorded, run the `lerobot-train` pipeline. By adding `--pretrained_policy_name_or_path`, you start from the organizer's model and fine-tune it. Setting `--push_to_hub=true` automatically uploads the final model so you can share it!
   ```bash
   pixi run lerobot-train \
     --dataset.repo_id=YOUR_HF_USERNAME/aic_cable_dataset \
     --policy.type=act \
     --pretrained_policy_name_or_path=grkw/aic_act_policy \
     --output_dir=outputs/train/my_act_model \
     --job_name=my_act_model \
     --policy.device=cuda \
     --push_to_hub=true \
     --hub_model_id=YOUR_HF_USERNAME/my_act_model
   ```

3. **Integration into a Policy**: 
   You don't need to write the inference code from scratch! 
   - Copy `aic_example_policies/aic_example_policies/ros/RunACT.py` into your `my_policy_node` folder.
   - Open your copied `RunACT.py`. Around line 65, change `policy_path` to point directly to your local trained model directory (e.g., `policy_path = Path("outputs/train/my_act_model")`) instead of downloading from HuggingFace.
   - Run the simulation and launch your modified `RunACT` policy to see your model control the robot!

## To-Do List
- [ ] Implement OpenCV image processing inside `estimate_port_pose_from_image`
- [ ] Implement spiral force-search compliance in the descent loop
- [ ] Tune Z-force thresholds for the specific cable/port

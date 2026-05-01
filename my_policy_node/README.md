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

## To-Do List
- [ ] Implement OpenCV image processing inside `estimate_port_pose_from_image`
- [ ] Implement spiral force-search compliance in the descent loop
- [ ] Tune Z-force thresholds for the specific cable/port

# Workflow B: End-to-End Imitation Learning with LeRobot (ACT)

## Overview

Workflow B is an **end-to-end imitation learning** approach for solving the cable insertion task on the AIC UR5e robot. Rather than decomposing the problem into perception + planning + control (Workflow A), this approach learns a direct mapping from **raw camera images + robot state → velocity commands** using the **ACT (Action Chunking with Transformers)** policy architecture.

The core idea: a human demonstrates the task many times via teleoperation, and a neural network learns to replicate that behavior.

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        WORKFLOW B PIPELINE                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────┐  │
│  │ Phase 1      │    │ Phase 2      │    │ Phase 3                  │  │
│  │ Data         │───▶│ Model        │───▶│ Inference / Deployment   │  │
│  │ Generation   │    │ Training     │    │ (RunACT Policy Node)     │  │
│  └──────────────┘    └──────────────┘    └──────────────────────────┘  │
│        │                    │                        │                   │
│   Teleoperation        Fine-tune ACT          ROS 2 control loop        │
│   via keyboard/        on collected            at ~4 Hz                  │
│   SpaceMouse           demonstrations                                   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

| Requirement | Details |
|-------------|---------|
| Environment Manager | [pixi](https://prefix.dev/tools/pixi) (manages all dependencies) |
| LeRobot Version | `0.5.1` (pinned in `pixi.toml`) |
| Pretrained Model | `grkw/aic_act_policy` on Hugging Face |
| GPU | CUDA-capable GPU recommended for training; inference can run on CPU |
| Hugging Face Account | Required for dataset/model upload (`huggingface-cli login`) |
| ROS 2 | Kilted distribution (managed by pixi/robostack) |

---

## Phase 1: Data Generation (Teleoperation & Recording)

### What Happens

A human operator controls the robot in real-time to solve the cable insertion task. Each successful demonstration is recorded as an episode in a LeRobot-compatible HDF5 dataset containing synchronized camera images, robot state, and the operator's actions.

### Components Involved

| Component | File | Purpose |
|-----------|------|---------|
| Robot Driver | `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_robot_aic_controller.py` | Bridges LeRobot's `Robot` interface to ROS 2 topics |
| Teleop Drivers | `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_teleop.py` | Keyboard & SpaceMouse input → action dicts |
| Robot Config | `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_robot.py` | Camera topics, joint names |
| Type Definitions | `aic_utils/lerobot_robot_aic/lerobot_robot_aic/types.py` | Action TypedDicts |

### Observation Space (What Gets Recorded)

#### Cameras (3 × RGB)

| Camera | ROS Topic | Native Resolution | Recorded Resolution (0.25×) |
|--------|-----------|-------------------|----------------------------|
| Left | `/left_camera/image` | 1152 × 1024 | 288 × 256 |
| Center | `/center_camera/image` | 1152 × 1024 | 288 × 256 |
| Right | `/right_camera/image` | 1152 × 1024 | 288 × 256 |

#### Robot State (26-dimensional vector)

| Index | Field | Dimensions |
|-------|-------|-----------|
| 0–2 | TCP Position (x, y, z) | 3 |
| 3–6 | TCP Orientation quaternion (x, y, z, w) | 4 |
| 7–9 | TCP Linear Velocity (x, y, z) | 3 |
| 10–12 | TCP Angular Velocity (x, y, z) | 3 |
| 13–18 | TCP Error (x, y, z, rx, ry, rz) | 6 |
| 19–25 | Joint Positions (shoulder_pan → wrist_3 + gripper) | 7 |

### Action Space (What the Operator Commands)

**Cartesian mode** (`MotionUpdateActionDict` — 6 floats):

| Key | Meaning |
|-----|---------|
| `linear.x` | X velocity (m/s) |
| `linear.y` | Y velocity (m/s) |
| `linear.z` | Z velocity (m/s) |
| `angular.x` | Roll rate (rad/s) |
| `angular.y` | Pitch rate (rad/s) |
| `angular.z` | Yaw rate (rad/s) |

**Joint mode** (`JointMotionUpdateActionDict` — 6 floats):

| Key | Meaning |
|-----|---------|
| `shoulder_pan_joint` | Joint velocity (rad/s) |
| `shoulder_lift_joint` | Joint velocity (rad/s) |
| `elbow_joint` | Joint velocity (rad/s) |
| `wrist_1_joint` | Joint velocity (rad/s) |
| `wrist_2_joint` | Joint velocity (rad/s) |
| `wrist_3_joint` | Joint velocity (rad/s) |

### Teleop Options

| `--teleop.type` | `--robot.teleop_target_mode` | Input Device | Control Space |
|-----------------|------------------------------|--------------|---------------|
| `aic_keyboard_ee` | `cartesian` | Keyboard | Cartesian (end-effector) |
| `aic_spacemouse` | `cartesian` | 3Dconnexion SpaceMouse | Cartesian (end-effector) |
| `aic_keyboard_joint` | `joint` | Keyboard | Joint space |

### Recording Command

```bash
cd ~/ws_aic/src/aic

# Log in to Hugging Face first
huggingface-cli login

# Record demonstrations
pixi run lerobot-record \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=aic_keyboard_ee --teleop.id=aic \
  --robot.teleop_target_mode=cartesian --robot.teleop_frame_id=base_link \
  --dataset.repo_id=YOUR_HF_USERNAME/aic_cable_dataset \
  --dataset.single_task="insert the cable" \
  --dataset.push_to_hub=true \
  --dataset.private=true \
  --play_sounds=false \
  --display_data=true
```

### Recording Controls

| Key | Action |
|-----|--------|
| Right Arrow | Finish current episode, start next |
| Left Arrow | Discard current episode, re-record |
| ESC | Stop recording entirely |

### Tips for Good Data

- Aim for **50+ demonstrations** minimum; more is better.
- Keep demonstrations consistent — start from similar poses.
- Vary the task board position slightly between episodes to improve generalization.
- Discard failed attempts (Left Arrow) rather than including bad data.
- The `--robot.teleop_frame_id=base_link` option sends commands relative to the robot base (recommended for consistency).

---

## Phase 2: Model Training (Fine-Tuning ACT)

### What Happens

The ACT (Action Chunking with Transformers) policy is fine-tuned on your collected demonstrations. ACT predicts a **chunk** of future actions at each timestep, which provides temporal smoothness and reduces compounding errors.

### Training Command

```bash
cd ~/ws_aic/src/aic

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

### Key Parameters

| Parameter | Purpose | Recommended Value |
|-----------|---------|-------------------|
| `--pretrained_policy_name_or_path` | Start from organizer's pretrained weights | `grkw/aic_act_policy` |
| `--policy.type` | Policy architecture | `act` |
| `--policy.device` | Training device | `cuda` |
| `--output_dir` | Local checkpoint directory | `outputs/train/my_act_model` |
| `--push_to_hub` | Auto-upload final model to HF | `true` |
| `--wandb.enable` | Enable Weights & Biases logging | `true` (optional) |

### Training Outputs

After training completes, the output directory contains:

```
outputs/train/my_act_model/
├── config.json                                              # ACT architecture config
├── model.safetensors                                        # Trained weights
└── policy_preprocessor_step_3_normalizer_processor.safetensors  # Normalization stats
```

These three files are everything needed for inference.

---

## Phase 3: Inference & Deployment (RunACT Policy Node)

### What Happens

The trained ACT model runs inside a ROS 2 policy node. At ~4 Hz, it reads camera images and robot state, runs a forward pass through the transformer, and publishes Cartesian velocity commands to the impedance controller.

### Reference Implementation

**File:** `aic_example_policies/aic_example_policies/ros/RunACT.py`

### Inference Pipeline (per timestep)

```
┌─────────────────────────────────────────────────────────────────────┐
│                     SINGLE INFERENCE STEP (~250ms)                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. GET OBSERVATION (from ROS topics)                               │
│     ├── 3 camera images (sensor_msgs/Image)                         │
│     ├── Controller state (tcp_pose, tcp_velocity, tcp_error)        │
│     └── Joint states (7 joint positions)                            │
│                                                                     │
│  2. PREPROCESS & NORMALIZE                                          │
│     ├── Images: resize 0.25× → HWC→CHW → /255 → (x-mean)/std      │
│     └── State: 26-dim vector → (x-mean)/std                        │
│                                                                     │
│  3. MODEL FORWARD PASS                                              │
│     └── ACTPolicy.select_action(obs) → normalized_action [1, 7]    │
│                                                                     │
│  4. UN-NORMALIZE ACTION                                             │
│     └── raw_action = (normalized × std) + mean                     │
│                                                                     │
│  5. PUBLISH COMMAND                                                 │
│     └── MotionUpdate msg (Cartesian twist + impedance params)       │
│         ├── frame_id: "base_link"                                   │
│         ├── stiffness: diag([100, 100, 100, 50, 50, 50])           │
│         ├── damping: diag([40, 40, 40, 15, 15, 15])                │
│         ├── wrench_feedback_gains: [0.5, 0.5, 0.5, 0, 0, 0]       │
│         └── mode: MODE_VELOCITY                                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Normalization Details

All inputs and outputs are normalized using statistics computed from the training dataset:

| Data | Normalization Formula | Stats Shape |
|------|----------------------|-------------|
| Camera images | `(pixel/255.0 - mean) / std` | `(1, 3, 1, 1)` per camera |
| Robot state | `(state - mean) / std` | `(1, 26)` |
| Action (output) | `action × std + mean` (un-normalize) | `(1, 7)` |

### Impedance Controller Parameters

The policy sends velocity commands through an impedance controller with these default gains:

| Parameter | Values | Units |
|-----------|--------|-------|
| Target Stiffness | `[100, 100, 100, 50, 50, 50]` | N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad |
| Target Damping | `[40, 40, 40, 15, 15, 15]` | Ns/m, Ns/m, Ns/m, Nms/rad, Nms/rad, Nms/rad |
| Wrench Feedback Gains | `[0.5, 0.5, 0.5, 0, 0, 0]` | dimensionless |
| Trajectory Mode | `MODE_VELOCITY` | — |

### ROS 2 Topics Used at Inference

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/left_camera/image` | `sensor_msgs/Image` | Subscribe | Left camera feed |
| `/center_camera/image` | `sensor_msgs/Image` | Subscribe | Center camera feed |
| `/right_camera/image` | `sensor_msgs/Image` | Subscribe | Right camera feed |
| `/aic_controller/controller_state` | `ControllerState` | Subscribe | TCP pose, velocity, error |
| `/joint_states` | `sensor_msgs/JointState` | Subscribe | Joint positions |
| `/aic_controller/pose_commands` | `MotionUpdate` | Publish | Velocity commands |
| `/aic_controller/change_target_mode` | `ChangeTargetMode` | Service | Switch to cartesian mode |

---

## Integration: Using Your Trained Model

### Option A: Point RunACT to Your Local Checkpoint

1. Copy `aic_example_policies/aic_example_policies/ros/RunACT.py` into `my_policy_node/my_policy_node/`.
2. Edit line ~65 to load from your local path instead of downloading:

```python
# Before (downloads from HuggingFace):
policy_path = Path(snapshot_download(repo_id=repo_id, ...))

# After (loads from local directory):
policy_path = Path("outputs/train/my_act_model")
```

3. Reinstall and run:

```bash
pixi reinstall ros-kilted-my-policy-node

pixi run ros2 run aic_model aic_model \
  --ros-args -p use_sim_time:=true \
  -p policy:=my_policy_node.RunACT
```

### Option B: Upload to HuggingFace and Download at Runtime

If you trained with `--push_to_hub=true`, change the `repo_id` in RunACT:

```python
repo_id = "YOUR_HF_USERNAME/my_act_model"
```

---

## File Reference

### Core Files

| File | Description |
|------|-------------|
| `aic_example_policies/aic_example_policies/ros/RunACT.py` | Reference inference node (copy & modify) |
| `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_robot_aic_controller.py` | LeRobot `Robot` interface for data collection |
| `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_teleop.py` | Teleoperator implementations (keyboard, SpaceMouse) |
| `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_robot.py` | Camera configs (topics, resolution) and joint names |
| `aic_utils/lerobot_robot_aic/lerobot_robot_aic/types.py` | `MotionUpdateActionDict`, `JointMotionUpdateActionDict` |
| `aic_utils/lerobot_robot_aic/README.md` | LeRobot usage documentation |
| `my_policy_node/README.md` | Project README with both workflow descriptions |
| `pixi.toml` | Dependency management (LeRobot 0.5.1, ROS 2 Kilted) |

### Class Hierarchy

```
lerobot.robots.Robot (abstract)
└── AICRobotAICController
    ├── connect()          → starts ROS 2 node, subscribes to topics
    ├── get_observation()  → returns cameras + 26-dim state
    ├── send_action()      → publishes MotionUpdate or JointMotionUpdate
    └── disconnect()       → cleans up ROS 2 resources

lerobot.teleoperators.Teleoperator (abstract)
├── AICKeyboardEETeleop      → cartesian keyboard control
├── AICKeyboardJointTeleop   → joint-space keyboard control
└── AICSpaceMouseTeleop      → cartesian SpaceMouse control

aic_model.policy.Policy (abstract)
└── RunACT
    ├── __init__()           → loads model weights + normalization stats
    ├── prepare_observations() → ROS msg → normalized tensors
    ├── insert_cable()       → main control loop (30s @ 4Hz)
    └── set_cartesian_twist_target() → builds MotionUpdate msg
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `Waiting for service 'aic_controller/change_target_mode'...` | The AIC controller isn't running. Start the simulation first. |
| `KeyError: 'linear.x'` | Mismatch between `--teleop.type` and `--robot.teleop_target_mode`. Use `cartesian` with `aic_keyboard_ee`/`aic_spacemouse`. |
| `KeyError: 'shoulder_pan_joint'` | Use `--robot.teleop_target_mode=joint` with `aic_keyboard_joint`. |
| Camera returns all-black images | Camera not ready yet; the driver uses a black placeholder. Wait a moment. |
| `Watchdog Validator` warnings from zenoh | Safe to ignore. Look for `Recording episode 0` to confirm recording started. |
| Model outputs erratic actions | Check that `image_scaling` (0.25) matches what was used during training. Verify normalization stats file exists. |
| SpaceMouse not detected | Add udev rules for 3Dconnexion (vendor `046d`) and reload with `udevadm`. |

---

## Summary: End-to-End Workflow Commands

```bash
# 0. Enter environment
cd ~/ws_aic/src/aic
huggingface-cli login

# 1. Record demonstrations (repeat many times for good data)
pixi run lerobot-record \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=aic_keyboard_ee --teleop.id=aic \
  --robot.teleop_target_mode=cartesian --robot.teleop_frame_id=base_link \
  --dataset.repo_id=YOUR_HF_USERNAME/aic_cable_dataset \
  --dataset.single_task="insert the cable" \
  --dataset.push_to_hub=true \
  --display_data=true

# 2. Train (fine-tune pretrained ACT)
pixi run lerobot-train \
  --dataset.repo_id=YOUR_HF_USERNAME/aic_cable_dataset \
  --policy.type=act \
  --pretrained_policy_name_or_path=grkw/aic_act_policy \
  --output_dir=outputs/train/my_act_model \
  --job_name=my_act_model \
  --policy.device=cuda \
  --push_to_hub=true \
  --hub_model_id=YOUR_HF_USERNAME/my_act_model

# 3. Deploy (run inference)
pixi reinstall ros-kilted-my-policy-node
pixi run ros2 run aic_model aic_model \
  --ros-args -p use_sim_time:=true \
  -p policy:=my_policy_node.RunACT
```

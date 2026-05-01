# GenerateData.py — Detailed Code Walkthrough

This document walks through every section of `GenerateData.py`, explaining **why** each piece of code exists and **how** it works.

---

## Overview

`GenerateData.py` is a **ROS 2 Policy Node** that, instead of inserting a cable, drives the robot arm around the task board in a loop and automatically saves perfectly labelled YOLO-Pose training images to disk. It replaces the need for any manual annotation.

The output is saved to `~/aic/yolo_dataset/` in a format ready to train with `ultralytics` (YOLO) directly.

```
yolo_dataset/
├── images/
│   ├── 00000.jpg
│   ├── 00001.jpg
│   └── ...
└── labels/
    ├── 00000.txt   ← one YOLO line per visible port
    ├── 00001.txt
    └── ...
```

---

## Part 1: Imports & Service Detection (Lines 18–43)

```python
try:
    from aic_training_interfaces.srv import ExpandXacro
    from simulation_interfaces.srv import DeleteEntity, SpawnEntity
    HAS_SERVICES = True
except ImportError:
    HAS_SERVICES = False
```

The three Gazebo services needed for dynamic board randomization may not always be installed. The `try/except` block makes the script **gracefully degrade**: if the services aren't available, it still collects data — just without randomizing the board layout. This prevents a hard crash.

---

## Part 2: `__init__` — Setup (Lines 47–68)

```python
home_dir = os.path.expanduser("~")
self._dataset_path = os.path.join(home_dir, "aic", "yolo_dataset")
```

The dataset path is constructed dynamically from the user's home directory so it works on any machine without hardcoding paths.

```python
os.makedirs(self._images_path, exist_ok=True)
os.makedirs(self._labels_path, exist_ok=True)
```

These two lines ensure the `images/` and `labels/` folders always exist before the script tries to write to them. Without this, `cv2.imwrite()` would crash silently on the first image.

```python
self._images_since_respawn = 0
```

This counter tracks how many images have been saved since the last board randomization. When it reaches 25, the board is respawned with a new random layout.

---

## Part 3: `_wait_for_tf` — Safety Helper (Lines 70–95)

```python
def _wait_for_tf(self, target_frame, source_frame, timeout_sec=10.0) -> bool:
```

The ROS 2 TF tree (the system that knows where every object in the simulation is) takes a moment to populate after the simulation starts or after a board is respawned. This helper function **polls in a loop** until the requested transform appears, or returns `False` after a timeout.

This prevents the main loop from crashing when it tries to look up a port that hasn't appeared in the TF tree yet.

---

## Part 4: `get_random_hover_pose` — Arm Positioning (Lines 97–159)

This method figures out a randomized pose for the robot arm to move to above the port. The goal is to make the arm hover at many different angles and distances so the training images have **diverse viewpoints**.

```python
q_diff = quaternion_multiply(q_port, q_plug_inv)
q_gripper_target = quaternion_multiply(q_diff, q_gripper)
```

This is quaternion math to calculate the rotation the gripper needs to face towards the port correctly. It works by:
1. Finding the angular difference between the plug's current orientation and the port's orientation.
2. Applying that difference to the gripper's current orientation to get the target orientation.

```python
target_x = port_transform.translation.x + random.uniform(-0.06, 0.06)
target_y = port_transform.translation.y + random.uniform(-0.06, 0.06)
target_z = port_transform.translation.z + random.uniform(0.12, 0.25) - offset[2]
```

The arm is positioned randomly ±6 cm in X and Y (left/right/forward/back) and between 12 cm and 25 cm above the port in Z. This creates **diverse viewing angles** in the training images, which is critical for a robust model.

---

## Part 5: `quat_to_mat` — Math Utility (Lines 161–168)

```python
def quat_to_mat(self, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([...])
```

Converts a ROS `Quaternion` message (which describes a 3D rotation using 4 numbers) into a standard 3×3 **Rotation Matrix**. The rotation matrix is needed for the camera projection math in Part 7. This is a standard formula from linear algebra.

---

## Part 6: `randomize_board` — Scene Randomization (Lines 170–223)

This is the core of the dataset diversity strategy. Every 25 images, this method is called to respawn the task board with a completely new random configuration.

### Step 1: Delete
```python
req_del = DeleteEntity.Request()
req_del.name = "task_board"
```
Sends a request to Gazebo's physics server to remove the existing task board from the simulation entirely.

### Step 2: Expand Xacro
```python
req_expand.package_name = "aic_description"
req_expand.relative_path = "urdf/task_board.urdf.xacro"
req_expand.xacro_arguments = args
```

The task board's physical description is stored as a **Xacro** template file (a parameterized XML/URDF format). We call the `/expand_xacro` service with randomized arguments to generate new physics XML on the fly:

```python
yaw = random.uniform(-0.5, 0.5)        # random board rotation ~±30°
nic_card_mount_0_present = random ...  # 70% chance each NIC slot is present
sfp_mount_rail_0_present  = random ...
```

### Step 3: Spawn
```python
req_spawn.xml = xml
req_spawn.initial_pose.position.x = 0.35 + random.uniform(-0.05, 0.05)
```

Sends the freshly generated XML to Gazebo to spawn the new randomized board, also at a slightly randomized position.

---

## Part 7: `insert_cable` — The Main Data Collection Loop (Lines 225–383)

Despite the name `insert_cable` (inherited from the `Policy` base class interface), this method **never inserts the cable**. Instead it runs an infinite data collection loop.

### Setup

```python
port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
```

This constructs the TF frame name of the **target port** from the `Task` message sent by the simulation engine. For example: `task_board/nic_card_mount_0/port_0_link`.

```python
possible_modules = ["nic_card_mount_0", ..., "sfp_mount_rail_0", ...]
possible_ports   = ["port_0_link", ..., "sfp_port_0_link", ..., "sc_port_0_link", ...]
```

These lists enumerate every possible port that *could* exist on the board in any configuration. The inner loop will check each one against the live TF tree to see if it actually exists right now.

### The Main Loop

Every iteration of `while True:` does the following:

#### Step 1 — Randomize if needed
```python
if self._images_since_respawn >= 25:
    self.randomize_board()
```
Triggers a full board respawn every 25 images.

#### Step 2 — Move the Arm
```python
target_pose = self.get_random_hover_pose(port_transform)
self.set_pose_target(move_robot=move_robot, pose=target_pose)
self.sleep_for(1.5)
```
Commands the robot arm to move to a new random position above the port and waits 1.5 seconds for it to physically get there and stop vibrating.

#### Step 3 — Get Camera Image
```python
obs = get_observation()
camera_frame = obs.center_camera_info.header.frame_id
```
Reads the latest snapshot of all sensors from the simulation. `camera_frame` is the TF frame name of the camera's optical centre, needed for the projection math.

#### Step 4 — Define Projection Constants
```python
corners_local = np.array([
    [-0.007, -0.005, 0.0],
    [ 0.007, -0.005, 0.0],
    [ 0.007,  0.005, 0.0],
    [-0.007,  0.005, 0.0],
])
K      = np.array(obs.center_camera_info.k).reshape(3, 3)
width  = float(obs.center_camera_info.width)
height = float(obs.center_camera_info.height)
```

- **`corners_local`**: The 4 physical corners of a port opening, measured in metres in the port's own local coordinate frame. An SFP port is roughly 14mm × 10mm.
- **`K`**: The camera **intrinsic matrix** — a 3×3 matrix that encodes the camera's focal length and optical centre. This is the "recipe" that converts 3D coordinates into 2D pixel coordinates for this specific camera.
- **`width` / `height`**: The image resolution in pixels (1152×1024 for this camera), needed to normalize the pixel coordinates to the [0, 1] range that YOLO requires.

#### Step 5 — Project Every Port to 2D (The Core Math)

```python
for module in possible_modules:
    for port in possible_ports:
        frame = f"task_board/{module}/{port}"
        if not tf_buffer.can_transform(camera_frame, frame, Time()):
            continue
```

For every possible port in the scene, the script first checks if that TF frame exists (i.e., if that port is actually present on the current board). If it does, it proceeds:

```python
T = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
R = self.quat_to_mat(tf.rotation)
corners_cam = [R @ pt + T for pt in corners_local]
```

**This is the key transformation.** For each of the 4 corner points:
- `R @ pt` rotates the corner from the port's local frame into the camera's frame.
- `+ T` translates it by the 3D position of the port relative to the camera.
- Result: `corners_cam` contains the 4 corners in **3D camera space** (in metres, relative to the camera lens).

```python
uv = K @ pt       # multiply by intrinsic matrix
u  = uv[0] / uv[2]   # divide by Z (perspective projection)
v  = uv[1] / uv[2]
u_norm = np.clip(u / width,  0.0, 1.0)
v_norm = np.clip(v / height, 0.0, 1.0)
```

**Pinhole camera projection.** Multiplying a 3D point by the intrinsic matrix `K` and then dividing by the Z coordinate gives the exact 2D pixel location of that 3D point in the image. The result is then normalized to [0, 1] by dividing by image width/height.

#### Step 6 — Format the YOLO Label

```python
cx = (max(us) + min(us)) / 2.0   # bounding box centre X
cy = (max(vs) + min(vs)) / 2.0   # bounding box centre Y
w  = max(us) - min(us)            # bounding box width
h  = max(vs) - min(vs)            # bounding box height

line = f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"
for p in pixels:
    line += f" {p[0]:.6f} {p[1]:.6f} 2"   # keypoint x y visibility
```

YOLO-Pose `.txt` format requires one line per object:
`class_id  cx  cy  w  h  kp1_x  kp1_y  kp1_v  kp2_x  kp2_y  kp2_v  ...`

- All coordinates are normalized to [0, 1].
- `class_id = 0` since we only have one class ("port").
- The trailing `2` for each keypoint means **"visible"** in YOLO's keypoint visibility convention (0=missing, 1=hidden, 2=visible).

#### Step 7 — Save to Disk

```python
img_bgr = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
cv2.imwrite(img_filename, img_bgr)
with open(lbl_filename, "w") as f:
    f.write("\n".join(yolo_lines) + "\n")
```

Saves the image (converting from ROS's RGB format to OpenCV's BGR format) and writes all the YOLO label lines to a matching `.txt` file. One `.txt` file can contain multiple lines — one per visible port in that image.

---

## How to Run

```bash
# 1. Rebuild the package
pixi reinstall ros-kilted-my-policy-node

# 2. Launch the simulation with training utils (exposes /expand_xacro)
pixi run ros2 launch aic_training_utils aic_training_gz_bringup.launch.py \
    spawn_task_board:=true spawn_cable:=true ground_truth:=true

# 3. Run the data collection policy
pixi run ros2 run aic_model aic_model \
    --ros-args -p use_sim_time:=true -p policy:=my_policy_node.GenerateData
```

> [!NOTE]
> `ground_truth:=true` is **required** — this is what populates the TF tree with the port locations that the script reads. Without it, `_wait_for_tf` will time out and no data will be collected.

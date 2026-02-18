# AIC MuJoCo Integration

This package provides documentation, scripts, and utilities for training policies in MuJoCo simulation using the AI for Industry Challenge (AIC) environment.

## Overview

[MuJoCo](https://mujoco.org/) is a physics engine designed for research and development in robotics, biomechanics, graphics and animation. In collaboration with **Google DeepMind**, this integration enables participants to:

- Convert Gazebo SDF worlds to MuJoCo MJCF format using `sdformat_mjcf`
- Load the AIC task board and robot from exported Gazebo worlds (`/tmp/aic.sdf`)
- Train policies using MuJoCo's fast physics simulation
- Control the UR5e robot using the same `aic_controller` interface
- Leverage domain randomization across multiple simulators (Gazebo, MuJoCo, IsaacLab)

## Workflow Summary

1. **Export from Gazebo**: Launch `aic_gz_bringup` with desired domain randomization parameters
2. **Automatic SDF Export**: Gazebo saves complete world to `/tmp/aic.sdf`
3. **Convert to MJCF**: Use `sdformat_mjcf` tool to convert SDF → MuJoCo XML + mesh assets
4. **Manual Refinements**: Apply necessary MJCF fixes (e.g., `shell="0"` for certain geometries)
5. **Load in MuJoCo**: Open the converted `scene.xml` in MuJoCo viewer or simulation
6. **Train Policy**: Use same control interface as Gazebo
7. **Validate**: Test trained policy back in Gazebo before submission

## Prerequisites

- **Operating System:** Ubuntu 24.04 (Noble Numbat)
- **ROS 2:** ROS 2 Kilted Kaiju
- **Existing AIC Workspace:** Follow the [Getting Started](../../docs/getting_started.md) guide to set up your base workspace

> **Note:** MuJoCo integration requires a native Ubuntu 24.04 installation with ROS 2 built from source or installed via apt. The pixi-based workflow does not support the necessary ROS 2 Control packages but the converted scene may be opened in MuJoCo installed in the pixi environment (via drag-and-drop or Python script - see [Loading in MuJoCo](#loading-in-mujoco)).

## Installation

### 1. Import MuJoCo Dependencies

From your ROS 2 workspace, import the required repositories:

```bash
cd ~/ws_aic/src
vcs import < aic/aic_utils/aic_mujoco/mujoco.repos
```

This adds:
- `mujoco_vendor` (v0.0.6) - ROS 2 wrapper for MuJoCo 3.x
- `mujoco_ros2_control` - Integration between MuJoCo and ros2_control
- `gz-mujoco` (with `sdformat_mjcf` tool) - Converts Gazebo SDF files to MuJoCo MJCF format

### 2. Install Dependencies

Install dependencies for the newly imported MuJoCo packages:

```bash
cd ~/ws_aic
rosdep install --from-paths src/gazebo/gz-mujoco src/mujoco --ignore-src -r -y
```

Or install all workspace dependencies:

```bash
cd ~/ws_aic
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

With the package dependencies properly configured, building should work automatically:

```bash
source /opt/ros/kilted/setup.bash

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

The build order is automatically handled by `colcon` through package dependencies:
1. `mujoco_vendor` builds first and exports `MUJOCO_DIR` via environment hooks
2. `mujoco_ros2_control` and `gz-mujoco` build next, finding MuJoCo automatically
3. Other packages build as needed

> **Note:** The forked repositories include fixes to ensure `mujoco_vendor` exports `MUJOCO_DIR` and `mujoco_ros2_control` properly depends on `mujoco_vendor`.

### 4. Verify Installation

```bash
# Source the workspace (if not already done)
source ~/ws_aic/install/setup.bash

# Check MUJOCO_DIR is automatically set by the environment hook
echo $MUJOCO_DIR
# Should output something like:
# /home/user/ws_aic/install/opt/mujoco_vendor

# Check MuJoCo installation directory
ls $MUJOCO_DIR
# Should show: bin, include, lib, simulate directories

# Check sdformat_mjcf tool
which sdformat_mjcf
# Should output:
# /home/user/ws_aic/install/gz-mujoco/bin/sdformat_mjcf

# Verify MuJoCo simulate binary works
which simulate
# Should output:
# /home/user/ws_aic/install/opt/mujoco_vendor/bin/simulate
```

> **⚠️ Important:** If you have a previous MuJoCo installation, it may conflict with `mujoco_vendor`. Check for and remove any existing `MUJOCO_PATH` or `MUJOCO_DIR` environment variables from your shell configuration (`~/.bashrc`, etc.) before building. After cleaning the environment, rebuild the workspace:
> ```bash
> # Check for conflicting environment variables
> env | grep MUJOCO
>
> # If you see MUJOCO_PATH or incorrect MUJOCO_DIR, remove from ~/.bashrc and restart shell
> # Then rebuild
> colcon build --packages-select mujoco_vendor mujoco_ros2_control gz-mujoco
> ```

## Usage

### Exporting the AIC World from Gazebo

First, generate the world state from Gazebo with your desired configuration:

```bash
source ~/ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'

# Example: Spawn task board with cable
ros2 launch aic_bringup aic_gz_bringup.launch.py \
  spawn_task_board:=true \
  task_board_x:=0.3 task_board_y:=-0.1 task_board_z:=1.2 \
  spawn_cable:=true \
  cable_type:=sfp_sc_cable \
  attach_cable_to_gripper:=true \
  ground_truth:=true

# The world is automatically exported to /tmp/aic.sdf
```

### Converting SDF to MJCF

Use the `sdformat_mjcf` tool to convert the exported SDF file to MuJoCo format:

```bash
source ~/ws_aic/install/setup.bash

# Convert SDF to MJCF (creates MJCF XML files and extracts mesh assets)
sdformat_mjcf /tmp/aic.sdf --output-dir ~/aic_mujoco_world

# Or use the convenience script:
python3 aic_utils/aic_mujoco/scripts/convert_world.py /tmp/aic.sdf ~/aic_mujoco_world
```

This creates:
- `scene.xml` - Main MuJoCo scene file
- `aic_robot.xml` - Robot model
- `*.stl` files - Extracted mesh geometry
- `aic_world.xml` - World configuration

### Manual MJCF Refinements

After conversion, some manual fixes may be needed:

1. **Shell geometries**: Set `shell="0"` for certain mesh geometries to avoid rendering artifacts
2. **Contact parameters**: Adjust friction, damping for cable behavior
3. **Visual refinements**: Material colors, textures

These refinements will be automated in future releases.

### Loading in MuJoCo

#### Using pixi environment

The Python viewer starts in **paused mode by default**. Press Space to start/pause simulation.

```bash
# Enter pixi shell
pixi shell

# Option 1: Launch empty viewer (then drag and drop scene.xml into the window)
python -m mujoco.viewer

# Option 2: Use the provided convenience script (starts paused)
python src/aic/aic_utils/aic_mujoco/scripts/view_scene.py ~/aic_mujoco_world/scene.xml

# Option 3: Use a one-liner Python command (paused mode)
python -c "import mujoco, mujoco.viewer; m = mujoco.MjModel.from_xml_path('~/aic_mujoco_world/scene.xml'); d = mujoco.MjData(m); v = mujoco.viewer.launch_passive(m, d); v.sync(); exec('while v.is_running(): v.sync()')"
```

> **Tip:** Press Space in the viewer to start/pause simulation, Backspace to reset.

#### Using native ROS 2 workspace

The `simulate` binary (from `mujoco_vendor`) can load scenes directly from command line and starts **paused by default**:

```bash
# Load scene (paused by default)
simulate ~/aic_mujoco_world/scene.xml
```

> **Tip:** Press Space to start/pause simulation in the viewer.

## Training with MuJoCo

MuJoCo's integration with ros2_control allows you to use the same `aic_controller` interface as in Gazebo, ensuring your policy code remains simulator-agnostic.

Key benefits for training:
- **Faster simulation** than Gazebo for large-scale training
- **Domain randomization** across Gazebo, MuJoCo, and IsaacLab
- **Same control interface** as evaluation environment

## Directory Structure

```
aic_mujoco/
├── README.md              # This file
├── mujoco.repos          # VCS repositories for MuJoCo dependencies
├── scripts/              # Utility scripts (to be added)
├── examples/             # Example training setups (to be added)
└── docs/                 # Additional documentation (to be added)
```

## Tested Configuration

This integration has been tested with:
- **mujoco_vendor:** v0.0.6 (MuJoCo 3.x)
- **mujoco_ros2_control:** commit `c811dcca2039cf6a88af3077e3f3dbbbd9c37a66`
- **gz-mujoco:** branch `fix_mesh_uri_handling`
- **ROS 2:** Kilted Kaiju
- **Ubuntu:** 24.04 LTS

## Known Limitations

- **SDF Conversion**: Not all Gazebo features convert perfectly (e.g., some plugins, advanced materials)
- **Manual Fixes Required**: Some MJCF refinements needed post-conversion (e.g., `shell="0"` for geometries)
- **Cable Physics**: Deformable cable behavior differs from Gazebo - intentional for domain randomization
- **Mesh URIs**: Ensure all mesh paths are resolved correctly during conversion
- **Contact Models**: May need tuning for insertion tasks

## Future Automation

Planned improvements to automate the workflow:
- Automated post-conversion MJCF fixes
- Pre-configured contact parameters for cable insertion
- One-command conversion script
- Docker container with complete MuJoCo training environment
- ROS 2 launch files for MuJoCo simulation with aic_controller

## Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [mujoco_ros2_control GitHub](https://github.com/ros-controls/mujoco_ros2_control)
- [AIC Getting Started Guide](../../docs/getting_started.md)
- [AIC Scene Description](../../docs/scene_description.md)

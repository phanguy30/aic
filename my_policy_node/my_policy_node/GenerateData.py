#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#


import os
import random
import time
import numpy as np
import cv2

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

try:
    from aic_training_interfaces.srv import ExpandXacro
    from simulation_interfaces.srv import DeleteEntity, SpawnEntity
    HAS_SERVICES = True
except ImportError:
    HAS_SERVICES = False


class GenerateData(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        
        # Dynamic paths based on home directory
        home_dir = os.path.expanduser("~")
        self._dataset_path = os.path.join(home_dir, "aic", "yolo_dataset")
        self._images_path = os.path.join(self._dataset_path, "images")
        self._labels_path = os.path.join(self._dataset_path, "labels")
        self._image_count = 0
        self._images_since_respawn = 0
        
        # Ensure directories exist
        os.makedirs(self._images_path, exist_ok=True)
        os.makedirs(self._labels_path, exist_ok=True)
        
        if HAS_SERVICES:
            self._cli_expand = self._parent_node.create_client(ExpandXacro, '/expand_xacro')
            self._cli_delete = self._parent_node.create_client(DeleteEntity, '/gz_server/delete_entity')
            self._cli_spawn = self._parent_node.create_client(SpawnEntity, '/gz_server/spawn_entity')
            self.get_logger().info("Successfully loaded randomization services!")
        else:
            self.get_logger().warn("Randomization services not found. Will not randomize board.")

    def _wait_for_tf(
        self, target_frame: str, source_frame: str, timeout_sec: float = 10.0
    ) -> bool:
        """Wait for a TF frame to become available."""
        start = self.time_now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                )
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(
                        f"Waiting for transform '{source_frame}' -> '{target_frame}'..."
                    )
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(
            f"Transform '{source_frame}' not available after {timeout_sec}s"
        )
        return False

    def get_random_hover_pose(self, port_transform: Transform) -> Pose:
        """Generate a slightly randomized pose above the port."""
        q_port = (
            port_transform.rotation.w,
            port_transform.rotation.x,
            port_transform.rotation.y,
            port_transform.rotation.z,
        )
        plug_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            f"{self._task.cable_name}/{self._task.plug_name}_link",
            Time(),
        )
        q_plug = (
            plug_tf_stamped.transform.rotation.w,
            plug_tf_stamped.transform.rotation.x,
            plug_tf_stamped.transform.rotation.y,
            plug_tf_stamped.transform.rotation.z,
        )
        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        
        gripper_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
            "base_link", "gripper/tcp", Time()
        )
        q_gripper = (
            gripper_tf_stamped.transform.rotation.w,
            gripper_tf_stamped.transform.rotation.x,
            gripper_tf_stamped.transform.rotation.y,
            gripper_tf_stamped.transform.rotation.z,
        )
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)
        
        plug_xyz = (
            plug_tf_stamped.transform.translation.x,
            plug_tf_stamped.transform.translation.y,
            plug_tf_stamped.transform.translation.z,
        )
        gripper_xyz = (
            gripper_tf_stamped.transform.translation.x,
            gripper_tf_stamped.transform.translation.y,
            gripper_tf_stamped.transform.translation.z,
        )
        offset = (
            gripper_xyz[0] - plug_xyz[0],
            gripper_xyz[1] - plug_xyz[1],
            gripper_xyz[2] - plug_xyz[2],
        )

        # Randomize X and Y by +/- 6cm, Z by 12cm to 25cm
        target_x = port_transform.translation.x + random.uniform(-0.06, 0.06)
        target_y = port_transform.translation.y + random.uniform(-0.06, 0.06)
        target_z = port_transform.translation.z + random.uniform(0.12, 0.25) - offset[2]

        return Pose(
            position=Point(x=target_x, y=target_y, z=target_z),
            orientation=Quaternion(
                w=q_gripper_target[0],
                x=q_gripper_target[1],
                y=q_gripper_target[2],
                z=q_gripper_target[3],
            ),
        )

    def quat_to_mat(self, q):
        """Convert ROS Quaternion to 3x3 Rotation Matrix."""
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])

    def randomize_board(self):
        """Delete current board and spawn a randomized new one."""
        if not HAS_SERVICES:
            return
            
        self.get_logger().info("Randomizing Task Board...")
        
        # 1. Delete Entity
        req_del = DeleteEntity.Request()
        req_del.name = "task_board"
        future = self._cli_delete.call_async(req_del)
        self._parent_node.executor.spin_until_future_complete(future, timeout_sec=2.0)
        
        self.sleep_for(1.0)
        
        # 2. Expand Xacro with random layout
        req_expand = ExpandXacro.Request()
        req_expand.package_name = "aic_description"
        req_expand.relative_path = "urdf/task_board.urdf.xacro"
        
        yaw = random.uniform(-0.5, 0.5)
        args = [
            "ground_truth:=true",
            f"task_board_yaw:={yaw}",
            f"nic_card_mount_0_present:={'true' if random.random() > 0.3 else 'false'}",
            f"nic_card_mount_1_present:={'true' if random.random() > 0.3 else 'false'}",
            f"nic_card_mount_2_present:={'true' if random.random() > 0.3 else 'false'}",
            f"nic_card_mount_3_present:={'true' if random.random() > 0.3 else 'false'}",
            f"sfp_mount_rail_0_present:={'true' if random.random() > 0.3 else 'false'}",
            f"sc_mount_rail_0_present:={'true' if random.random() > 0.3 else 'false'}"
        ]
        req_expand.xacro_arguments = args
        
        future = self._cli_expand.call_async(req_expand)
        self._parent_node.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.result() or not future.result().success:
            self.get_logger().error("ExpandXacro failed!")
            return
            
        xml = future.result().xml
        
        # 3. Spawn Entity
        req_spawn = SpawnEntity.Request()
        req_spawn.name = "task_board"
        req_spawn.xml = xml
        req_spawn.initial_pose = Pose()
        req_spawn.initial_pose.position.x = 0.35 + random.uniform(-0.05, 0.05)
        req_spawn.initial_pose.position.y = random.uniform(-0.05, 0.05)
        req_spawn.initial_pose.position.z = 1.05
        
        future = self._cli_spawn.call_async(req_spawn)
        self._parent_node.executor.spin_until_future_complete(future, timeout_sec=5.0)
        self.sleep_for(2.0)
        self.get_logger().info("New Task Board Spawned!")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info("Starting YOLO Data Generation Policy!")
        self._task = task
        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        cable_tip_frame = f"{task.cable_name}/{task.plug_name}_link"

        for frame in [port_frame, cable_tip_frame]:
            if not self._wait_for_tf("base_link", frame):
                return False

        try:
            port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link", port_frame, Time()
            )
            port_transform = port_tf_stamped.transform
        except TransformException as ex:
            self.get_logger().error(f"Could not look up port transform: {ex}")
            return False

        # Define possible modules and port names to search for
        possible_modules = [
            "nic_card_mount_0", "nic_card_mount_1", "nic_card_mount_2", "nic_card_mount_3", "nic_card_mount_4",
            "sc_mount_rail_0", "sc_mount_rail_1", "sfp_mount_rail_0", "sfp_mount_rail_1"
        ]
        possible_ports = [f"port_{i}_link" for i in range(5)] + [f"sfp_port_{i}_link" for i in range(5)] + [f"sc_port_{i}_link" for i in range(5)]
        
        # Loop indefinitely to collect data
        while True:
            # Randomize board every 25 images
            if self._images_since_respawn >= 25:
                self.randomize_board()
                self._images_since_respawn = 0
                
                # Re-check main target frame after respawn
                if not self._wait_for_tf("base_link", port_frame):
                    continue
                    
                try:
                    port_tf_stamped = self._parent_node._tf_buffer.lookup_transform("base_link", port_frame, Time())
                    port_transform = port_tf_stamped.transform
                except TransformException:
                    continue

            # 1. Move to random pose
            target_pose = self.get_random_hover_pose(port_transform)
            try:
                self.set_pose_target(move_robot=move_robot, pose=target_pose)
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed during interpolation: {ex}")
            
            # Wait for robot to move there and stabilize
            self.sleep_for(1.5)

            # 2. Get observation
            obs = get_observation()
            if obs is None:
                continue

            camera_frame = obs.center_camera_info.header.frame_id
            if not self._wait_for_tf(camera_frame, port_frame, timeout_sec=2.0):
                continue
            
            # --- Define constants needed inside the port projection loop ---
            # Physical size of port in meters (SFP port is ~14mm x 10mm)
            corners_local = np.array([
                [-0.007, -0.005, 0.0],
                [ 0.007, -0.005, 0.0],
                [ 0.007,  0.005, 0.0],
                [-0.007,  0.005, 0.0],
            ])

            # Camera intrinsic matrix from camera_info
            K = np.array(obs.center_camera_info.k).reshape(3, 3)
            width = float(obs.center_camera_info.width)
            height = float(obs.center_camera_info.height)

            # Iterate through all possible ports on the board
            yolo_lines = []
            
            for module in possible_modules:
                for port in possible_ports:
                    frame = f"task_board/{module}/{port}"
                    if not self._parent_node._tf_buffer.can_transform(camera_frame, frame, Time()):
                        continue
                        
                    try:
                        cam_port_tf = self._parent_node._tf_buffer.lookup_transform(
                            camera_frame, frame, Time()
                        )
                    except TransformException:
                        continue

                    T = np.array([
                        cam_port_tf.transform.translation.x,
                        cam_port_tf.transform.translation.y,
                        cam_port_tf.transform.translation.z
                    ])
                    R = self.quat_to_mat(cam_port_tf.transform.rotation)

                    corners_cam = [R @ pt + T for pt in corners_local]

                    pixels = []
                    for pt in corners_cam:
                        if pt[2] <= 0:
                            break
                        uv = K @ pt
                        u = uv[0] / uv[2]
                        v = uv[1] / uv[2]
                        u_norm = np.clip(u / width, 0.0, 1.0)
                        v_norm = np.clip(v / height, 0.0, 1.0)
                        pixels.append((u_norm, v_norm))
                    
                    if len(pixels) != 4:
                        continue

                    # Check if all points are within image bounds
                    if any(p[0] <= 0 or p[0] >= 1 or p[1] <= 0 or p[1] >= 1 for p in pixels):
                        continue

                    us = [p[0] for p in pixels]
                    vs = [p[1] for p in pixels]
                    cx = (max(us) + min(us)) / 2.0
                    cy = (max(vs) + min(vs)) / 2.0
                    w = max(us) - min(us)
                    h = max(vs) - min(vs)

                    class_id = 0
                    line = f"{class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"
                    for p in pixels:
                        line += f" {p[0]:.6f} {p[1]:.6f} 2"
                    
                    yolo_lines.append(line)

            if not yolo_lines:
                self.get_logger().warn("No ports visible in image, skipping...")
                continue

            # 4. Save Image and Label
            img_data = np.frombuffer(obs.center_image.data, dtype=np.uint8).reshape(
                (obs.center_image.height, obs.center_image.width, 3)
            )
            img_bgr = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)

            img_filename = os.path.join(self._images_path, f"{self._image_count:05d}.jpg")
            lbl_filename = os.path.join(self._labels_path, f"{self._image_count:05d}.txt")

            cv2.imwrite(img_filename, img_bgr)
            with open(lbl_filename, "w") as f:
                f.write("\n".join(yolo_lines) + "\n")

            self.get_logger().info(f"Saved generated image {self._image_count:05d} with {len(yolo_lines)} labeled ports.")
            self._image_count += 1
            self._images_since_respawn += 1

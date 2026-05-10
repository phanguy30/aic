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

from aic_control_interfaces.srv import ChangeTargetMode
import time
import numpy as np

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_control_interfaces.msg import (
    MotionUpdate,
    TrajectoryGenerationMode,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Wrench
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException


class InsertCable(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("InsertCable initialized.")

        self._mode_client = parent_node.create_client(
            ChangeTargetMode, '/aic_controller/change_target_mode'
        )

    def _wait_for_tf(
        self, target_frame: str, source_frame: str, timeout_sec: float = 10.0
    ) -> bool:
        start = self.time_now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self.time_now() - start) < timeout:
            try:
                self._parent_node._tf_buffer.lookup_transform(
                    target_frame, source_frame, Time()
                )
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(
                        f"Waiting for TF '{source_frame}' -> '{target_frame}'..."
                    )
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(
            f"TF '{source_frame}' not available after {timeout_sec}s"
        )
        return False

    def _get_port_pose_from_tf(self, task: Task) -> Pose | None:
        """Phase 1: Get real port position using ground truth TF."""
        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"

        if not self._wait_for_tf("base_link", port_frame):
            return None

        try:
            tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link", port_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"TF lookup failed: {ex}")
            return None

        t = tf_stamped.transform
        return Pose(
            position=Point(
                x=t.translation.x,
                y=t.translation.y,
                z=t.translation.z,
            ),
            orientation=Quaternion(
                x=t.rotation.x,
                y=t.rotation.y,
                z=t.rotation.z,
                w=t.rotation.w,
            ),
        )

    def _approach(
        self,
        target_pose: Pose,
        move_robot: MoveRobotCallback,
        hover_z: float = 0.2,
    ) -> None:
        """Phase 2: Smoothly move to hover_z above the target port."""
        hover_pose = Pose(
            position=Point(
                x=target_pose.position.x,
                y=target_pose.position.y,
                z=hover_z,
            ),
            orientation=target_pose.orientation,
        )

        for t in range(100):
            frac = t / 100.0
            interp_pose = Pose(
                position=Point(
                    x=frac * hover_pose.position.x + (1.0 - frac) * target_pose.position.x,
                    y=frac * hover_pose.position.y + (1.0 - frac) * target_pose.position.y,
                    z=frac * hover_pose.position.z + (1.0 - frac) * target_pose.position.z,
                ),
                orientation=hover_pose.orientation,
            )
            self.set_pose_target(move_robot=move_robot, pose=interp_pose)
            self.sleep_for(0.05)

        self.sleep_for(1.0)

    def _spiral_search(
        self,
        target_pose: Pose,
        current_z: float,
        move_robot: MoveRobotCallback,
        radius: float = 0.005,
        steps: int = 12,
    ) -> None:
        """Circular search pattern when force is detected."""
        for i in range(steps):
            angle = 2.0 * np.pi * i / steps
            search_pose = Pose(
                position=Point(
                    x=target_pose.position.x + radius * np.cos(angle),
                    y=target_pose.position.y + radius * np.sin(angle),
                    z=current_z,
                ),
                orientation=target_pose.orientation,
            )
            self.set_pose_target(move_robot=move_robot, pose=search_pose)
            self.sleep_for(0.05)

    def _descend_and_insert(
        self,
        target_pose: Pose,
        move_robot: MoveRobotCallback,
        get_observation: GetObservationCallback,
        send_feedback: SendFeedbackCallback,
        start_z: float = 0.2,
        end_z: float = -0.015,
        step: float = 0.0005,
        force_threshold: float = 15.0,
    ) -> bool:
        """Phase 3: Descend 0.5mm at a time while monitoring wrist force."""
        current_z = start_z

        while current_z > end_z:
            observation = get_observation()
            if observation is None:
                continue

            z_force = observation.wrist_wrench.wrench.force.z
            self.get_logger().info(f"z: {current_z:.4f}  z_force: {z_force:.2f}N")

            if abs(z_force) > force_threshold:
                send_feedback(f"Resistance detected ({z_force:.1f}N) - running spiral search")
                self.get_logger().warn(f"Resistance detected: {z_force:.2f}N. Starting spiral search.")
                current_z += 0.005  # Back off 5mm

                if current_z > 0.35:
                    self.get_logger().error(
                        f"CRITICAL ABORT: Arm reached unsafe height ({current_z:.3f}m)."
                    )
                    send_feedback("Mission Aborted: Safety ceiling breached.")
                    return False

                self._spiral_search(target_pose, current_z, move_robot)
            else:
                current_z -= step
                self.set_pose_target(
                    move_robot=move_robot,
                    pose=Pose(
                        position=Point(
                            x=target_pose.position.x,
                            y=target_pose.position.y,
                            z=current_z,
                        ),
                        orientation=target_pose.orientation,
                    ),
                )

            self.sleep_for(0.05)

        return True

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(
            f"insert_cable() started: {task.target_module_name}/{task.port_name}"
        )

        # Switch to Cartesian mode (non-blocking, best-effort)
        if self._mode_client.wait_for_service(timeout_sec=2.0):
            req = ChangeTargetMode.Request()
            req.target_mode.mode = 1
            self._mode_client.call_async(req)
            self.get_logger().info("Cartesian Target Mode requested.")
        else:
            self.get_logger().warn("mode service not available, skipping Cartesian switch")

        # Phase 1: Get real port position from ground truth TF
        send_feedback("Phase 1: Getting port position from TF")
        target_pose = self._get_port_pose_from_tf(task)
        if target_pose is None:
            self.get_logger().error("Failed to get port pose from TF!")
            return False
        self.get_logger().info(
            f"Port position: x={target_pose.position.x:.3f} "
            f"y={target_pose.position.y:.3f} z={target_pose.position.z:.3f}"
        )

        # Phase 2: Smoothly approach hover position
        send_feedback("Phase 2: Approaching hover position")
        self.get_logger().info("Phase 2 started: approach")
        self._approach(target_pose, move_robot, hover_z=0.2)

        # Phase 3: Descend and insert
        send_feedback("Phase 3: Insertion started")
        self.get_logger().info("Phase 3 started: descent and insertion")
        success = self._descend_and_insert(
            target_pose=target_pose,
            move_robot=move_robot,
            get_observation=get_observation,
            send_feedback=send_feedback,
        )

        if success:
            send_feedback("Insertion complete!")
            self.get_logger().info("Insertion successful. Waiting to stabilize...")
            self.sleep_for(3.0)

        self.get_logger().info("insert_cable() finished.")
        return success

    def set_pose_target(self, move_robot: MoveRobotCallback, pose: Pose) -> None:
        """Construct and send the Cartesian command with impedance parameters."""
        msg = MotionUpdate()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        msg.target_stiffness = [85.0, 85.0, 85.0, 85.0, 85.0, 85.0]
        msg.target_damping = [75.0, 75.0, 75.0, 75.0, 75.0, 75.0]
        msg.trajectory_generation_mode.mode = 2
        move_robot(msg)

    def sleep_for(self, seconds: float) -> None:
        time.sleep(seconds)

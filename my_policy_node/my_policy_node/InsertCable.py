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

"""
InsertCable Policy — Hybrid approach combining:
- CheatCode's proven quaternion alignment and PI drift correction
- Force-monitored descent with Archimedean spiral search fallback

Requires: ground_truth:=true (uses TF for port localization)
"""

import numpy as np

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


class InsertCable(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0
        self._max_integrator_windup = 0.05
        self._task = None
        self.get_logger().info("InsertCable policy initialized.")

    # -------------------------------------------------------------------------
    # Utility: Wait for TF
    # -------------------------------------------------------------------------

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
                    target_frame, source_frame, Time()
                )
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self.get_logger().info(
                        f"Waiting for TF '{source_frame}' -> '{target_frame}'... "
                        f"(ensure ground_truth:=true)"
                    )
                attempt += 1
                self.sleep_for(0.1)
        self.get_logger().error(
            f"TF '{source_frame}' not available after {timeout_sec}s"
        )
        return False

    # -------------------------------------------------------------------------
    # Core: Calculate gripper pose with orientation alignment + PI correction
    # (Ported from CheatCode.py — proven to work)
    # -------------------------------------------------------------------------

    def _calc_gripper_pose(
        self,
        port_transform: Transform,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        reset_xy_integrator: bool = False,
    ) -> Pose:
        """
        Calculate the gripper pose that aligns the plug with the port.

        Uses quaternion math to compute the rotation needed, and a PI controller
        to correct X/Y drift between the plug tip and port center.
        """
        # Port orientation
        q_port = (
            port_transform.rotation.w,
            port_transform.rotation.x,
            port_transform.rotation.y,
            port_transform.rotation.z,
        )

        # Current plug orientation
        plug_tf = self._parent_node._tf_buffer.lookup_transform(
            "base_link",
            f"{self._task.cable_name}/{self._task.plug_name}_link",
            Time(),
        )
        q_plug = (
            plug_tf.transform.rotation.w,
            plug_tf.transform.rotation.x,
            plug_tf.transform.rotation.y,
            plug_tf.transform.rotation.z,
        )
        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])

        # Rotation difference: how much to rotate gripper so plug matches port
        q_diff = quaternion_multiply(q_port, q_plug_inv)

        # Current gripper orientation
        gripper_tf = self._parent_node._tf_buffer.lookup_transform(
            "base_link", "gripper/tcp", Time()
        )
        q_gripper = (
            gripper_tf.transform.rotation.w,
            gripper_tf.transform.rotation.x,
            gripper_tf.transform.rotation.y,
            gripper_tf.transform.rotation.z,
        )

        # Target gripper orientation (apply rotation diff)
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)

        # Smooth rotation via SLERP
        q_gripper_slerp = quaternion_slerp(q_gripper, q_gripper_target, slerp_fraction)

        # Position calculations
        gripper_xyz = (
            gripper_tf.transform.translation.x,
            gripper_tf.transform.translation.y,
            gripper_tf.transform.translation.z,
        )
        port_xy = (
            port_transform.translation.x,
            port_transform.translation.y,
        )
        plug_xyz = (
            plug_tf.transform.translation.x,
            plug_tf.transform.translation.y,
            plug_tf.transform.translation.z,
        )

        # Offset between gripper frame and plug tip
        plug_tip_gripper_offset = (
            gripper_xyz[0] - plug_xyz[0],
            gripper_xyz[1] - plug_xyz[1],
            gripper_xyz[2] - plug_xyz[2],
        )

        # PI controller for X/Y drift correction
        tip_x_error = port_xy[0] - plug_xyz[0]
        tip_y_error = port_xy[1] - plug_xyz[1]

        if reset_xy_integrator:
            self._tip_x_error_integrator = 0.0
            self._tip_y_error_integrator = 0.0
        else:
            self._tip_x_error_integrator = np.clip(
                self._tip_x_error_integrator + tip_x_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )
            self._tip_y_error_integrator = np.clip(
                self._tip_y_error_integrator + tip_y_error,
                -self._max_integrator_windup,
                self._max_integrator_windup,
            )

        i_gain = 0.15
        target_x = port_xy[0] + i_gain * self._tip_x_error_integrator
        target_y = port_xy[1] + i_gain * self._tip_y_error_integrator
        target_z = port_transform.translation.z + z_offset - plug_tip_gripper_offset[2]

        # Blend from current position to target (for smooth approach)
        blend_xyz = (
            position_fraction * target_x + (1.0 - position_fraction) * gripper_xyz[0],
            position_fraction * target_y + (1.0 - position_fraction) * gripper_xyz[1],
            position_fraction * target_z + (1.0 - position_fraction) * gripper_xyz[2],
        )

        return Pose(
            position=Point(x=blend_xyz[0], y=blend_xyz[1], z=blend_xyz[2]),
            orientation=Quaternion(
                w=q_gripper_slerp[0],
                x=q_gripper_slerp[1],
                y=q_gripper_slerp[2],
                z=q_gripper_slerp[3],
            ),
        )

    # -------------------------------------------------------------------------
    # Phase 2: Smooth approach to hover position
    # -------------------------------------------------------------------------

    def _approach(
        self,
        port_transform: Transform,
        move_robot: MoveRobotCallback,
        z_offset: float = 0.2,
    ) -> None:
        """Smoothly interpolate from current pose to hover position above port."""
        self.get_logger().info(f"Approaching port (z_offset={z_offset}m)...")

        for t in range(100):
            frac = t / 100.0
            try:
                pose = self._calc_gripper_pose(
                    port_transform,
                    slerp_fraction=frac,
                    position_fraction=frac,
                    z_offset=z_offset,
                    reset_xy_integrator=True,
                )
                self.set_pose_target(move_robot=move_robot, pose=pose)
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed during approach: {ex}")
            self.sleep_for(0.05)

        # Stabilize at hover
        self.sleep_for(1.0)
        self.get_logger().info("Approach complete.")

    # -------------------------------------------------------------------------
    # Phase 3a: Archimedean spiral search
    # -------------------------------------------------------------------------

    def _spiral_search(
        self,
        port_transform: Transform,
        z_offset: float,
        move_robot: MoveRobotCallback,
        get_observation: GetObservationCallback,
        spiral_spacing: float = 0.002,
        max_radius: float = 0.012,
        angular_step: float = 0.15,
        force_threshold: float = 5.0,
    ) -> bool:
        """
        Archimedean spiral search: r(t) = a*t.

        Expands outward from current position. If Z-force drops below threshold,
        the plug has found the hole. Returns True if hole found, False if exhausted.
        """
        a = spiral_spacing / (2.0 * np.pi)
        theta = 0.0

        # Get current plug position as spiral center
        try:
            plug_tf = self._parent_node._tf_buffer.lookup_transform(
                "base_link",
                f"{self._task.cable_name}/{self._task.plug_name}_link",
                Time(),
            )
        except TransformException:
            return False

        cx = plug_tf.transform.translation.x
        cy = plug_tf.transform.translation.y

        self.get_logger().info(
            f"Spiral search: center=({cx:.4f}, {cy:.4f}), "
            f"max_r={max_radius*1000:.1f}mm, spacing={spiral_spacing*1000:.1f}mm"
        )

        while True:
            theta += angular_step
            r = a * theta

            if r > max_radius:
                self.get_logger().warn("Spiral search exhausted.")
                return False

            # Compute spiral offset and apply to gripper pose
            # We adjust the port_transform temporarily to shift the target
            offset_x = r * np.cos(theta)
            offset_y = r * np.sin(theta)

            # Create a shifted port transform for the spiral position
            shifted_transform = Transform()
            shifted_transform.translation.x = port_transform.translation.x + offset_x
            shifted_transform.translation.y = port_transform.translation.y + offset_y
            shifted_transform.translation.z = port_transform.translation.z
            shifted_transform.rotation = port_transform.rotation

            try:
                pose = self._calc_gripper_pose(
                    shifted_transform, z_offset=z_offset
                )
                self.set_pose_target(move_robot=move_robot, pose=pose)
            except TransformException:
                continue

            self.sleep_for(0.04)

            # Check force
            obs = get_observation()
            if obs is None:
                continue

            fz = abs(obs.wrist_wrench.wrench.force.z)
            if fz < force_threshold:
                self.get_logger().info(
                    f"Hole found! r={r*1000:.2f}mm, Fz={fz:.2f}N"
                )
                return True

        return False

    # -------------------------------------------------------------------------
    # Phase 3: Force-monitored descent with spiral search fallback
    # -------------------------------------------------------------------------

    def _descend_and_insert(
        self,
        port_transform: Transform,
        move_robot: MoveRobotCallback,
        get_observation: GetObservationCallback,
        send_feedback: SendFeedbackCallback,
        start_z_offset: float = 0.2,
        end_z_offset: float = -0.015,
        step: float = 0.0005,
        force_threshold: float = 15.0,
        timeout_sec: float = 30.0,
    ) -> bool:
        """
        Descend toward the port while monitoring wrist force.

        If excessive force is detected:
        1. Back off 3mm
        2. Run Archimedean spiral search
        3. If hole found, continue descent from new position
        4. If not found after 3 attempts, abort
        """
        z_offset = start_z_offset
        max_spiral_attempts = 3
        spiral_attempts = 0
        start_time = self.time_now()
        max_duration = Duration(seconds=timeout_sec)

        self.get_logger().info(
            f"Descent started: z_offset {start_z_offset} -> {end_z_offset}, "
            f"step={step*1000:.1f}mm, force_limit={force_threshold}N"
        )

        while z_offset > end_z_offset:
            # Safety timeout
            if (self.time_now() - start_time) > max_duration:
                self.get_logger().error("Descent timeout reached!")
                send_feedback("Timeout during insertion.")
                return False

            # Get force reading
            observation = get_observation()
            if observation is None:
                self.sleep_for(0.02)
                continue

            fz = abs(observation.wrist_wrench.wrench.force.z)

            if fz > force_threshold:
                # Resistance detected — back off and spiral search
                spiral_attempts += 1
                self.get_logger().warn(
                    f"Resistance: Fz={fz:.1f}N > {force_threshold}N. "
                    f"Spiral attempt {spiral_attempts}/{max_spiral_attempts}"
                )
                send_feedback(
                    f"Resistance ({fz:.1f}N) — spiral search "
                    f"{spiral_attempts}/{max_spiral_attempts}"
                )

                if spiral_attempts > max_spiral_attempts:
                    self.get_logger().error("Max spiral attempts exceeded. Aborting.")
                    send_feedback("Insertion failed: could not find hole.")
                    return False

                # Back off 3mm
                z_offset += 0.003
                if z_offset > 0.25:
                    z_offset = 0.25  # Safety cap

                try:
                    pose = self._calc_gripper_pose(port_transform, z_offset=z_offset)
                    self.set_pose_target(move_robot=move_robot, pose=pose)
                except TransformException:
                    pass
                self.sleep_for(0.3)

                # Run spiral search
                found = self._spiral_search(
                    port_transform, z_offset, move_robot, get_observation
                )
                if found:
                    spiral_attempts = 0  # Reset on success
                    self.get_logger().info("Spiral found hole, resuming descent.")
            else:
                # Normal descent
                z_offset -= step
                try:
                    pose = self._calc_gripper_pose(port_transform, z_offset=z_offset)
                    self.set_pose_target(move_robot=move_robot, pose=pose)
                except TransformException as ex:
                    self.get_logger().warn(f"TF failed during descent: {ex}")

            self.sleep_for(0.05)

        self.get_logger().info(f"Descent complete at z_offset={z_offset:.4f}")
        return True

    # -------------------------------------------------------------------------
    # Main entry point
    # -------------------------------------------------------------------------

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(
            f"InsertCable.insert_cable() — "
            f"target: {task.target_module_name}/{task.port_name}, "
            f"cable: {task.cable_name}/{task.plug_name}"
        )
        self._task = task

        port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        cable_tip_frame = f"{task.cable_name}/{task.plug_name}_link"

        # Wait for TF frames
        for frame in [port_frame, cable_tip_frame]:
            if not self._wait_for_tf("base_link", frame):
                return False

        # Look up port transform
        try:
            port_tf_stamped = self._parent_node._tf_buffer.lookup_transform(
                "base_link", port_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"Port TF lookup failed: {ex}")
            return False
        port_transform = port_tf_stamped.transform

        self.get_logger().info(
            f"Port at: ({port_transform.translation.x:.3f}, "
            f"{port_transform.translation.y:.3f}, "
            f"{port_transform.translation.z:.3f})"
        )

        # Phase 2: Smooth approach to 20cm above port
        send_feedback("Phase 2: Approaching port")
        self._approach(port_transform, move_robot, z_offset=0.2)

        # Phase 3: Force-monitored descent with spiral search
        send_feedback("Phase 3: Inserting cable")
        success = self._descend_and_insert(
            port_transform=port_transform,
            move_robot=move_robot,
            get_observation=get_observation,
            send_feedback=send_feedback,
        )

        if success:
            send_feedback("Insertion complete!")
            self.get_logger().info("Waiting for connector to stabilize...")
            self.sleep_for(5.0)
        else:
            send_feedback("Insertion failed.")

        self.get_logger().info("InsertCable.insert_cable() exiting.")
        return success

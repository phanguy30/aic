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


class WaveArm(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("WaveArm (Real Policy Skeleton) initialized.")

    def estimate_port_pose_from_image(self, observation: Observation) -> Pose:
        """
        Phase 1: Observation Gathering & Vision Pipeline Framework
        Dummy implementation to extract features from the center_image.
        """
        # Note: You can use cv_bridge to convert observation.center_image to an OpenCV image
        # cv_image = self.cv_bridge.imgmsg_to_cv2(observation.center_image, desired_encoding='bgr8')
        
        # TODO: Implement actual computer vision or DL model inference here
        # Example: template matching, color filtering, or passing to an ACT model.
        
        # Return a dummy target pose above the board
        return Pose(
            position=Point(x=-0.4, y=0.45, z=0.2), # Example coordinates
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        )

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ):
        self.get_logger().info(f"Starting real policy execution for task: {task.target_module_name}/{task.port_name}")
        send_feedback("Phase 1: Estimating port position")
        
        observation = get_observation()
        if not observation:
            self.get_logger().error("Failed to get initial observation!")
            return False

        # Phase 1: Estimate target
        target_pose = self.estimate_port_pose_from_image(observation)
        
        # Phase 2: Approach Trajectory
        send_feedback("Phase 2: Approaching hover position")
        self.get_logger().info("Approaching hover position...")
        # Move to hover pose (Z = 0.2 for example)
        hover_pose = Pose(
            position=Point(x=target_pose.position.x, y=target_pose.position.y, z=0.2),
            orientation=target_pose.orientation
        )
        self.set_pose_target(move_robot=move_robot, pose=hover_pose)
        self.sleep_for(2.0) # Wait for arm to settle
        
        # Phase 3: Insertion & Force-Compliance
        send_feedback("Phase 3: Descending and monitoring forces")
        current_z = hover_pose.position.z
        
        while current_z > 0.0: # Assuming 0.0 is roughly board level
            observation = get_observation()
            if not observation:
                continue
                
            # Check Z force
            z_force = observation.wrist_wrench.wrench.force.z
            
            # Simple thresholding
            if abs(z_force) > 15.0: # 15 Newtons threshold
                self.get_logger().warn(f"High resistance detected! Z-force: {z_force:.2f}. Adjusting...")
                send_feedback("High resistance - backing off")
                # TODO: Implement spiral search or compliance here
                # For now, just back off slightly
                current_z += 0.01
                self.sleep_for(0.5)
            else:
                # Continue descending
                current_z -= 0.005
                
            descend_pose = Pose(
                position=Point(x=target_pose.position.x, y=target_pose.position.y, z=current_z),
                orientation=target_pose.orientation
            )
            self.set_pose_target(move_robot=move_robot, pose=descend_pose)
            self.sleep_for(0.05) # ~20Hz control loop
            
        send_feedback("Insertion complete")
        self.get_logger().info("Policy execution finished.")
        return True

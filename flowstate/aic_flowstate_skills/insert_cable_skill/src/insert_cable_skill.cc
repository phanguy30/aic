/*
 * Copyright (C) 2026 Intrinsic Innovation LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "insert_cable_skill.h"

#include <chrono>

#include "absl/status/statusor.h"
#include "aic_task_interfaces/action/insert_cable.hpp"
#include "insert_cable_skill.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using InsertCableAction = aic_task_interfaces::action::InsertCable;

//==============================================================================
// ROS initialization.
//==============================================================================

namespace {
class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

class InsertCableClientNode : public rclcpp::Node {
 public:
  InsertCableClientNode() : Node("insert_cable_skill_node") {
    client_ =
        rclcpp_action::create_client<InsertCableAction>(this, "/insert_cable");
  }

  bool WaitForServer(std::chrono::seconds timeout) {
    return client_->wait_for_action_server(timeout);
  }

  auto async_send_goal(
      const InsertCableAction::Goal& goal_msg,
      const rclcpp_action::Client<InsertCableAction>::SendGoalOptions&
          options) {
    return client_->async_send_goal(goal_msg, options);
  }

  auto async_get_result(
      const typename rclcpp_action::Client<
          InsertCableAction>::GoalHandle::SharedPtr& goal_handle) {
    return client_->async_get_result(goal_handle);
  }

  auto async_cancel_goal(
      const typename rclcpp_action::Client<
          InsertCableAction>::GoalHandle::SharedPtr& goal_handle) {
    return client_->async_cancel_goal(goal_handle);
  }

 private:
  rclcpp_action::Client<InsertCableAction>::SharedPtr client_;
};

InitRos init;
InsertCableClientNode client_node_;

std::mutex active_goal_mutex;
typename rclcpp_action::Client<InsertCableAction>::GoalHandle::SharedPtr
    active_goal_handle;
}  // namespace

#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

//==============================================================================
// Skill signature.
//==============================================================================

std::unique_ptr<intrinsic::skills::SkillInterface>
InsertCableSkill::CreateSkill() {
  return std::make_unique<InsertCableSkill>();
}
absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
InsertCableSkill::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                          intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

//==============================================================================
// Skill execution.
//==============================================================================

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
InsertCableSkill::Execute(const intrinsic::skills::ExecuteRequest& request,
                          intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(client_node_.get_logger(), "InsertCableSkill::Execute");

  // Get parameters.
  INTR_ASSIGN_OR_RETURN(
      auto params, request.params<ai::flowstate::InsertCableSkillParams>());

  // Wait for server
  if (!client_node_.WaitForServer(std::chrono::seconds(10))) {
    RCLCPP_ERROR(client_node_.get_logger(),
                 "Action server '/insert_cable' not available");
    return absl::InternalError("Action server not available");
  }

  // Populate goal msg
  auto goal_msg = InsertCableAction::Goal();
  goal_msg.task.id = params.id();
  goal_msg.task.cable_type = params.cable_type();
  goal_msg.task.cable_name = params.cable_name();
  goal_msg.task.plug_type = params.plug_type();
  goal_msg.task.plug_name = params.plug_name();
  goal_msg.task.port_type = params.port_type();
  goal_msg.task.port_name = params.port_name();
  goal_msg.task.target_module_name = params.target_module_name();
  goal_msg.task.time_limit = params.time_limit();

  RCLCPP_INFO(client_node_.get_logger(), "Sending goal for task ID: %s",
              goal_msg.task.id.c_str());

  auto send_goal_options =
      rclcpp_action::Client<InsertCableAction>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [](const rclcpp_action::ClientGoalHandle<InsertCableAction>::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(client_node_.get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(client_node_.get_logger(), "Goal accepted by server, waiting for result");
        }
      };

  send_goal_options.result_callback =
      [](const rclcpp_action::ClientGoalHandle<InsertCableAction>::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(client_node_.get_logger(), "Goal succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(client_node_.get_logger(), "Goal was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(client_node_.get_logger(), "Goal was canceled");
            break;
          default:
            RCLCPP_ERROR(client_node_.get_logger(), "Unknown result code");
            break;
        }
      };

  auto goal_handle_future =
      client_node_.async_send_goal(goal_msg, send_goal_options);

  // Spin to receive response
  if (rclcpp::spin_until_future_complete(client_node_.get_node_base_interface(),
                                         goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(client_node_.get_logger(), "Failed to send goal");
    return absl::InternalError("Failed to send goal");
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(client_node_.get_logger(), "Goal rejected by server");
    return absl::InternalError("Goal rejected by server");
  }

  // Store active handle thread-safely for concurrent cancellation halts
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex);
    active_goal_handle = goal_handle;
  }

  RCLCPP_INFO(client_node_.get_logger(),
              "Goal accepted, waiting for result...");

  auto result_future = client_node_.async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(client_node_.get_node_base_interface(),
                                         result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(client_node_.get_logger(), "Failed to get result");
    return absl::InternalError("Failed to get result");
  }

  // Clear active handle
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex);
    active_goal_handle.reset();
  }

  auto wrapped_result = result_future.get();

  RCLCPP_INFO(client_node_.get_logger(), "Task completed with success: %s",
              wrapped_result.result->success ? "true" : "false");

  auto result = std::make_unique<ai::flowstate::InsertCableSkillResult>();
  result->set_success(wrapped_result.result->success);
  result->set_message(wrapped_result.result->message);

  return result;
}

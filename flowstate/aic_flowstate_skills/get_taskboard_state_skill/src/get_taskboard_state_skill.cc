#include "get_taskboard_state_skill.h"

#include <chrono>
#include <mutex>
#include <sstream>

#include "absl/status/statusor.h"
#include "aic_flowstate_interface/srv/get_taskboard_state.hpp"
#include "get_taskboard_state_skill.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace {
class InitRos {
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

class GetTaskboardStateClientNode : public rclcpp::Node {
 public:
  GetTaskboardStateClientNode() : Node("get_taskboard_state_skill_node") {
    client_ =
        this->create_client<aic_flowstate_interface::srv::GetTaskboardState>(
            "get_taskboard_state");
  }

  absl::StatusOr<std::shared_ptr<
      aic_flowstate_interface::srv::GetTaskboardState::Response>>
  GetState(double timeout_ms) {
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      return absl::UnavailableError(
          "Service 'get_taskboard_state' not available");
    }

    auto request = std::make_shared<
        aic_flowstate_interface::srv::GetTaskboardState::Request>();
    auto result_future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), result_future,
            std::chrono::milliseconds(static_cast<int>(timeout_ms))) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return absl::DeadlineExceededError(
          "Timed out waiting for service response.");
    }

    return result_future.get();
  }

  rclcpp::Client<aic_flowstate_interface::srv::GetTaskboardState>::SharedPtr
      client_;
};

InitRos init;
GetTaskboardStateClientNode client_node_;

}  // namespace

//==============================================================================
// Skill signature.
//==============================================================================

std::unique_ptr<intrinsic::skills::SkillInterface>
GetTaskboardStateSkill::CreateSkill() {
  return std::make_unique<GetTaskboardStateSkill>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
GetTaskboardStateSkill::Preview(
    const intrinsic::skills::PreviewRequest& /*request*/,
    intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Skill has not implemented `Preview`.");
}

//==============================================================================
// Skill execution.
//==============================================================================

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
GetTaskboardStateSkill::Execute(
    const intrinsic::skills::ExecuteRequest& request,
    intrinsic::skills::ExecuteContext& /*context*/) {
  RCLCPP_INFO(client_node_.get_logger(), "GetTaskboardStateSkill::Execute");

  auto status_or_response = client_node_.GetState(5000.0);  // 5 seconds timeout

  if (!status_or_response.ok()) {
    return status_or_response.status();
  }

  auto response = status_or_response.value();
  auto result = std::make_unique<ai::flowstate::GetTaskboardStateSkillResult>();
  result->set_success(true);
  result->set_message("Successfully retrieved state");

  // Copy base pose
  result->set_task_board_x(response->task_board_x);
  result->set_task_board_y(response->task_board_y);
  result->set_task_board_z(response->task_board_z);
  result->set_task_board_roll(response->task_board_roll);
  result->set_task_board_pitch(response->task_board_pitch);
  result->set_task_board_yaw(response->task_board_yaw);

  // Helper lambda to copy component state
  auto copy_comp = [&](const auto& ros_comp, auto* proto_comp) {
    proto_comp->set_entity_present(ros_comp.entity_present);
    proto_comp->set_translation(ros_comp.translation);
    proto_comp->set_roll(ros_comp.roll);
    proto_comp->set_pitch(ros_comp.pitch);
    proto_comp->set_yaw(ros_comp.yaw);
  };

  copy_comp(response->nic_rail_0, result->mutable_nic_rail_0());
  copy_comp(response->nic_rail_1, result->mutable_nic_rail_1());
  copy_comp(response->nic_rail_2, result->mutable_nic_rail_2());
  copy_comp(response->nic_rail_3, result->mutable_nic_rail_3());
  copy_comp(response->nic_rail_4, result->mutable_nic_rail_4());
  copy_comp(response->sc_rail_0, result->mutable_sc_rail_0());
  copy_comp(response->sc_rail_1, result->mutable_sc_rail_1());
  copy_comp(response->lc_mount_rail_0, result->mutable_lc_mount_rail_0());
  copy_comp(response->sfp_mount_rail_0, result->mutable_sfp_mount_rail_0());
  copy_comp(response->sc_mount_rail_0, result->mutable_sc_mount_rail_0());
  copy_comp(response->lc_mount_rail_1, result->mutable_lc_mount_rail_1());
  copy_comp(response->sfp_mount_rail_1, result->mutable_sfp_mount_rail_1());
  copy_comp(response->sc_mount_rail_1, result->mutable_sc_mount_rail_1());

  return result;
}

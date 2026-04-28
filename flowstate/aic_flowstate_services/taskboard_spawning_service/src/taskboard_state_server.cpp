#include <chrono>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "aic_flowstate_interface/srv/get_taskboard_state.hpp"
#include "aic_flowstate_interface/srv/update_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace aic {
namespace flowstate {

class TaskboardStateServer : public rclcpp::Node {
 public:
  TaskboardStateServer() : Node("taskboard_state_server") {
    this->declare_parameter("config_file", "");
    std::string config_file = this->get_parameter("config_file").as_string();

    if (!config_file.empty()) {
      LoadDefaultState(config_file);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "No config file provided, starting with empty state.");
    }

    update_service_ =
        this->create_service<aic_flowstate_interface::srv::UpdateComponent>(
            "update_component",
            std::bind(&TaskboardStateServer::HandleUpdateComponent, this,
                      std::placeholders::_1, std::placeholders::_2));

    get_state_service_ =
        this->create_service<aic_flowstate_interface::srv::GetTaskboardState>(
            "get_taskboard_state",
            std::bind(&TaskboardStateServer::HandleGetTaskboardState, this,
                      std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Taskboard State Server ready.");
  }

 private:
  void LoadDefaultState(const std::string& file_path) {
    try {
      YAML::Node config = YAML::LoadFile(file_path);
      if (config["trials"]) {
        // We just take the first trial's scene as default state for now
        for (YAML::const_iterator it = config["trials"].begin();
             it != config["trials"].end(); ++it) {
          if (it->second["scene"] && it->second["scene"]["task_board"]) {
            state_node_ = it->second["scene"]["task_board"];
            RCLCPP_INFO(this->get_logger(), "Loaded default state from %s",
                        file_path.c_str());
            return;
          }
        }
      }
      RCLCPP_WARN(this->get_logger(), "No valid scene found in %s",
                  file_path.c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
    }
  }

  void HandleUpdateComponent(
      const std::shared_ptr<
          aic_flowstate_interface::srv::UpdateComponent::Request>
          request,
      std::shared_ptr<aic_flowstate_interface::srv::UpdateComponent::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "Received update for %s",
                request->component_name.c_str());

    YAML::Node comp;
    comp["entity_present"] = request->entity_present;
    comp["entity_name"] = request->entity_name;
    comp["entity_pose"]["translation"] = request->translation;
    comp["entity_pose"]["roll"] = request->roll;
    comp["entity_pose"]["pitch"] = request->pitch;
    comp["entity_pose"]["yaw"] = request->yaw;

    state_node_[request->component_name] = comp;

    response->success = true;
    response->message = "State updated successfully";
  }

  void HandleGetTaskboardState(
      const std::shared_ptr<
          aic_flowstate_interface::srv::GetTaskboardState::Request> /*request*/,
      std::shared_ptr<aic_flowstate_interface::srv::GetTaskboardState::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "State requested and served.");

    // Fill task_board base pose
    if (state_node_["pose"]) {
      response->task_board_x = state_node_["pose"]["x"]
                                   ? state_node_["pose"]["x"].as<double>()
                                   : 0.0;
      response->task_board_y = state_node_["pose"]["y"]
                                   ? state_node_["pose"]["y"].as<double>()
                                   : 0.0;
      response->task_board_z = state_node_["pose"]["z"]
                                   ? state_node_["pose"]["z"].as<double>()
                                   : 0.0;
      response->task_board_roll = state_node_["pose"]["roll"]
                                      ? state_node_["pose"]["roll"].as<double>()
                                      : 0.0;
      response->task_board_pitch =
          state_node_["pose"]["pitch"]
              ? state_node_["pose"]["pitch"].as<double>()
              : 0.0;
      response->task_board_yaw = state_node_["pose"]["yaw"]
                                     ? state_node_["pose"]["yaw"].as<double>()
                                     : 0.0;
    }

    // Helper lambda to fill component state
    auto fill_comp = [&](const std::string& key, auto& field) {
      if (state_node_[key]) {
        YAML::Node comp = state_node_[key];
        field.entity_present =
            comp["entity_present"] ? comp["entity_present"].as<bool>() : false;
        if (comp["entity_name"])
          field.entity_name = comp["entity_name"].as<std::string>();
        if (comp["entity_pose"]) {
          field.translation =
              comp["entity_pose"]["translation"]
                  ? comp["entity_pose"]["translation"].as<double>()
                  : 0.0;
          field.roll = comp["entity_pose"]["roll"]
                           ? comp["entity_pose"]["roll"].as<double>()
                           : 0.0;
          field.pitch = comp["entity_pose"]["pitch"]
                            ? comp["entity_pose"]["pitch"].as<double>()
                            : 0.0;
          field.yaw = comp["entity_pose"]["yaw"]
                          ? comp["entity_pose"]["yaw"].as<double>()
                          : 0.0;
        }
      }
    };

    fill_comp("nic_rail_0", response->nic_rail_0);
    fill_comp("nic_rail_1", response->nic_rail_1);
    fill_comp("nic_rail_2", response->nic_rail_2);
    fill_comp("nic_rail_3", response->nic_rail_3);
    fill_comp("nic_rail_4", response->nic_rail_4);
    fill_comp("sc_rail_0", response->sc_rail_0);
    fill_comp("sc_rail_1", response->sc_rail_1);
    fill_comp("lc_mount_rail_0", response->lc_mount_rail_0);
    fill_comp("sfp_mount_rail_0", response->sfp_mount_rail_0);
    fill_comp("sc_mount_rail_0", response->sc_mount_rail_0);
    fill_comp("lc_mount_rail_1", response->lc_mount_rail_1);
    fill_comp("sfp_mount_rail_1", response->sfp_mount_rail_1);
    fill_comp("sc_mount_rail_1", response->sc_mount_rail_1);
  }

  YAML::Node state_node_;
  rclcpp::Service<aic_flowstate_interface::srv::UpdateComponent>::SharedPtr
      update_service_;
  rclcpp::Service<aic_flowstate_interface::srv::GetTaskboardState>::SharedPtr
      get_state_service_;
};

}  // namespace flowstate
}  // namespace aic

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aic::flowstate::TaskboardStateServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

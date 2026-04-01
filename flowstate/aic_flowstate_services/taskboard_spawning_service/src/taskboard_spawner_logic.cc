#include "taskboard_spawning_service/taskboard_spawner_logic.h"

#include <iostream>
#include <cmath>
#include <algorithm>

namespace aic {
namespace flowstate {

bool TaskboardSpawnerLogic::LoadFromYaml(const std::string& yaml_file_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_file_path);
    return LoadFromYamlNode(config);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
    return false;
  }
}

bool TaskboardSpawnerLogic::LoadFromYamlNode(const YAML::Node& node) {
  YAML::Node target_node;

  if (node["trials"]) {
    // If it has "trials", find the first trial that has "scene"
    for (YAML::const_iterator it = node["trials"].begin(); it != node["trials"].end(); ++it) {
      if (it->second["scene"] && it->second["scene"]["task_board"]) {
        target_node = it->second;
        break;
      }
    }
  } else if (node["task_board_detector"] && node["task_board_detector"]["ros__parameters"]) {
    target_node = node["task_board_detector"]["ros__parameters"];
  } else if (node["scene"]) {
    target_node = node; // Standard root scene
  }

  if (!target_node["scene"] || !target_node["scene"]["task_board"]) {
    std::cerr << "Unknown YAML structure, missing valid scene/task_board" << std::endl;
    return false;
  }

  const YAML::Node& taskboard_node = target_node["scene"]["task_board"];

  components_.clear();
  InitializeDefaultLimits();

  // Iterate through potential component keys
  std::vector<std::string> keys = {
      "nic_rail_0", "nic_rail_1", "nic_rail_2", "nic_rail_3", "nic_rail_4",
      "sc_rail_0", "sc_rail_1",
      "lc_mount_rail_0", "sfp_mount_rail_0", "sc_mount_rail_0",
      "lc_mount_rail_1", "sfp_mount_rail_1", "sc_mount_rail_1"
  };

  for (const auto& key : keys) {
    if (taskboard_node[key]) {
      const YAML::Node& comp = taskboard_node[key];
      ComponentConfig config;
      config.entity_present = comp["entity_present"] && comp["entity_present"].as<bool>();
      if (config.entity_present) {
        if (comp["entity_name"]) {
          config.entity_name = comp["entity_name"].as<std::string>();
        } else {
          config.entity_name = key; // Fallback to key if name missing
        }
        if (comp["entity_pose"]) {
          const YAML::Node& pose = comp["entity_pose"];
          config.pose.translation = pose["translation"] ? pose["translation"].as<double>() : 0.0;
          config.pose.roll = pose["roll"] ? pose["roll"].as<double>() : 0.0;
          config.pose.pitch = pose["pitch"] ? pose["pitch"].as<double>() : 0.0;
          config.pose.yaw = pose["yaw"] ? pose["yaw"].as<double>() : 0.0;
        }
      }
      components_[key] = config;
    }
  }

  return true;
}

void TaskboardSpawnerLogic::InitializeDefaultLimits() {
  standard_limits_["nic_rail_0"] = Range{-0.048, 0.036};
  standard_limits_["nic_rail_1"] = Range{-0.048, 0.036};
  standard_limits_["nic_rail_2"] = Range{-0.048, 0.036};
  standard_limits_["nic_rail_3"] = Range{-0.048, 0.036};
  standard_limits_["nic_rail_4"] = Range{-0.048, 0.036};

  standard_limits_["sc_rail_0"] = Range{-0.055, 0.055};
  standard_limits_["sc_rail_1"] = Range{-0.055, 0.055};

  standard_limits_["lc_mount_rail_0"] = Range{-0.09625, 0.09625};
  standard_limits_["lc_mount_rail_1"] = Range{-0.09625, 0.09625};
  standard_limits_["sfp_mount_rail_0"] = Range{-0.09625, 0.09625};
  standard_limits_["sfp_mount_rail_1"] = Range{-0.09625, 0.09625};
  standard_limits_["sc_mount_rail_0"] = Range{-0.09625, 0.09625};
  standard_limits_["sc_mount_rail_1"] = Range{-0.09625, 0.09625};
}

bool TaskboardSpawnerLogic::IsWithinLimits(const std::string& component_name, double translation,
                                          double custom_min, double custom_max,
                                          bool use_custom_limits) const {
  Range limit;
  if (use_custom_limits) {
    limit = Range{custom_min, custom_max};
  } else {
    auto it = standard_limits_.find(component_name);
    if (it != standard_limits_.end()) {
      limit = it->second;
    } else {
      std::cerr << "Unknown limit for component " << component_name << std::endl;
      return false; // Or default to [-100, 100]? Better fail.
    }
  }

  return translation >= limit.min_val && translation <= limit.max_val;
}

bool TaskboardSpawnerLogic::GetUrdfBaseOffset(const std::string& component_name,
                                              double& offset_x, double& offset_y, double& offset_z,
                                              double& init_roll, double& init_pitch, double& init_yaw) const {
  // Hardcoded offsets from task_board.urdf.xacro
  if (component_name == "nic_rail_0") {
    offset_x = -0.081418; offset_y = -0.1745; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "nic_rail_1") {
    offset_x = -0.081418; offset_y = -0.1345; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "nic_rail_2") {
    offset_x = -0.081418; offset_y = -0.0945; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "nic_rail_3") {
    offset_x = -0.081418; offset_y = -0.0545; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "nic_rail_4") {
    offset_x = -0.081418; offset_y = -0.0145; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "sc_rail_0") {
    offset_x = -0.075; offset_y = 0.0295; offset_z = 0.0165;
    init_roll = 1.57; init_pitch = 0.0; init_yaw = 1.57;
    return true;
  } else if (component_name == "sc_rail_1") {
    offset_x = -0.075; offset_y = 0.0705; offset_z = 0.0165;
    init_roll = 1.57; init_pitch = 0.0; init_yaw = 1.57;
    return true;
  } else if (component_name == "lc_mount_rail_0") {
    offset_x = 0.01; offset_y = -0.10625; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "lc_mount_rail_1") {
    offset_x = 0.01; offset_y = 0.10625; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "sfp_mount_rail_0") {
    offset_x = 0.055; offset_y = -0.10625; offset_z = 0.01;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "sfp_mount_rail_1") {
    offset_x = 0.055; offset_y = 0.10625; offset_z = 0.01;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "sc_mount_rail_0") {
    offset_x = 0.1; offset_y = -0.10625; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  } else if (component_name == "sc_mount_rail_1") {
    offset_x = 0.0985; offset_y = 0.10625; offset_z = 0.012;
    init_roll = 0.0; init_pitch = 0.0; init_yaw = 0.0;
    return true;
  }

  return false;
}

bool TaskboardSpawnerLogic::ComputeRelativePose(
    const std::string& component_name,
    double& out_x, double& out_y, double& out_z,
    double& out_qx, double& out_qy, double& out_qz, double& out_qw) {

  ComponentConfig comp_config;
  auto it = components_.find(component_name);
  if (it != components_.end()) {
    comp_config = it->second;
  } else {
    std::cerr << "Component missing from YAML parsed config : " << component_name << std::endl;
    return false;
  }

  double urdf_offset_x, urdf_offset_y, urdf_offset_z;
  double init_roll, init_pitch, init_yaw;
  if (!GetUrdfBaseOffset(component_name, urdf_offset_x, urdf_offset_y, urdf_offset_z, init_roll, init_pitch, init_yaw)) {
    std::cerr << "Urdf offset missing for : " << component_name << std::endl;
    return false;
  }

  // Calculate translation in local frame
  double local_x = urdf_offset_x;
  double local_y = urdf_offset_y;
  double local_z = urdf_offset_z;

  if (component_name.find("nic_rail_") != std::string::npos ||
      component_name.find("sc_rail_") != std::string::npos) {
    local_x += comp_config.pose.translation; // NICs translate along X
  } else {
    local_y += comp_config.pose.translation; // Mount rails translate along Y
  }

  out_x = local_x;
  out_y = local_y;
  out_z = local_z;

  // Final orientation in local frame of base
  double final_roll = init_roll + comp_config.pose.roll;
  double final_pitch = init_pitch + comp_config.pose.pitch;
  double final_yaw = init_yaw + comp_config.pose.yaw;

  // Convert final_roll/pitch/yaw to Quaternion
  double cy = cos(final_yaw * 0.5);
  double sy = sin(final_yaw * 0.5);
  double cp = cos(final_pitch * 0.5);
  double sp = sin(final_pitch * 0.5);
  double cr = cos(final_roll * 0.5);
  double sr = sin(final_roll * 0.5);

  out_qw = cr * cp * cy + sr * sp * sy;
  out_qx = sr * cp * cy - cr * sp * sy;
  out_qy = cr * sp * cy + sr * cp * sy;
  out_qz = cr * cp * sy - sr * sp * cy;

  return true;
}

}  // namespace flowstate
}  // namespace aic

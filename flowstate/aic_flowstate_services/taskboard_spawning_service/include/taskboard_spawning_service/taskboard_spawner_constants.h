// Copyright 2026 Intrinsic Innovation LLC
// Confidential and Proprietary

#ifndef AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNING_SERVICE_TASKBOARD_SPAWNER_CONSTANTS_H_
#define AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNING_SERVICE_TASKBOARD_SPAWNER_CONSTANTS_H_

namespace aic {
namespace flowstate {

// Environment variables and defaults
constexpr char kAssetServerDefault[] = "localhost:17080";
// Asset metadata
constexpr char kPackageSideloaded[] = "ai.intrinsic";

// Topic and Service Names
constexpr char kUpdateComponentService[] = "/taskboard_spawner_service/update_component";

// ROS Parameters
constexpr char kParamConfigFile[] = "config_file_path";
constexpr char kParamBaseX[] = "taskboard_base_x";
constexpr char kParamBaseY[] = "taskboard_base_y";
constexpr char kParamBaseZ[] = "taskboard_base_z";
constexpr char kParamBaseYaw[] = "taskboard_base_yaw";

}  // namespace flowstate
}  // namespace aic

#endif  // AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNING_SERVICE_TASKBOARD_SPAWNER_CONSTANTS_H_

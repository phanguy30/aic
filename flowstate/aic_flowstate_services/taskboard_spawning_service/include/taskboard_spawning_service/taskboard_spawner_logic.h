#ifndef AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNER_LOGIC_H_
#define AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNER_LOGIC_H_

#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace aic {
namespace flowstate {

/// Structure defining an entity pose (from YAML)
struct EntityPose {
  double translation = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

/// Structure representing limit ranges
struct Range {
  double min_val = 0.0;
  double max_val = 0.0;
};

/// Structure representing a component state parsed from YAML
struct ComponentConfig {
  bool entity_present = false;
  std::string entity_name;
  EntityPose pose;
};

class TaskboardSpawnerLogic {
 public:
  TaskboardSpawnerLogic() = default;

  /// Load trial configuration from a YAML file or string.
  bool LoadFromYaml(const std::string& yaml_file_path);
  bool LoadFromYamlNode(const YAML::Node& node);

  /// Compute the relative pose for a child component based on its offsets from the base.
  /// \param component_name Key name of the rail component (e.g., "nic_rail_0", "lc_mount_rail_0").
  /// \param out_x, out_y, out_z, out_qx, out_qy, out_qz, out_qw The resulting relative pose.
  bool ComputeRelativePose(
      const std::string& component_name,
      double& out_x, double& out_y, double& out_z,
      double& out_qx, double& out_qy, double& out_qz, double& out_qw);

  /// Check if translation is within limits for a component.
  bool IsWithinLimits(const std::string& component_name, double translation,
                      double custom_min = 0.0, double custom_max = 0.0,
                      bool use_custom_limits = false) const;

  /// Update translation for a component.
  void UpdateTranslation(const std::string& component_name, double translation);

  /// Get the component config parsed from YAML.
  const std::unordered_map<std::string, ComponentConfig>& GetComponents() const {
    return components_;
  }

 private:
  std::unordered_map<std::string, ComponentConfig> components_;
  std::unordered_map<std::string, Range> standard_limits_;

  void InitializeDefaultLimits();
  bool GetUrdfBaseOffset(const std::string& component_name,
                         double& offset_x, double& offset_y, double& offset_z,
                         double& init_roll, double& init_pitch, double& init_yaw) const;
};

}  // namespace flowstate
}  // namespace aic

#endif  // AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNER_LOGIC_H_

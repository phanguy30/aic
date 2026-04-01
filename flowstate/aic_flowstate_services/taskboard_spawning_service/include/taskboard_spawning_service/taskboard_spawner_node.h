// Copyright 2026 Intrinsic Innovation LLC
// Confidential and Proprietary

#ifndef AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNING_SERVICE_TASKBOARD_SPAWNER_NODE_H_
#define AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNING_SERVICE_TASKBOARD_SPAWNER_NODE_H_

#include "taskboard_spawning_service/taskboard_spawner_logic.h"
#include <rclcpp/rclcpp.hpp>
#include "aic_flowstate_interface/srv/update_component.hpp"

#include <grpcpp/grpcpp.h>
#include "intrinsic/assets/proto/asset_deployment.grpc.pb.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "google/longrunning/operations.grpc.pb.h"

#include <memory>
#include <string>

namespace aic {
namespace flowstate {

class TaskboardSpawnerNode : public rclcpp::Node {
 public:
  TaskboardSpawnerNode();
  virtual ~TaskboardSpawnerNode() = default;

 private:
  void InitGrpcStubs();
  void SpawnFromConfig(const std::string& config_path);
  bool SpawnComponent(const std::string& component_type, const std::string& instance_name,
                      double x, double y, double z, double rx_or_qx, double ry_or_qy, double rz_or_qz, 
                      double rw = 1.0, bool use_quat = false, const std::string& parent_name = "");
  bool RemoveComponent(const std::string& entity_name);

  void HandleUpdateComponent(
      const std::shared_ptr<aic_flowstate_interface::srv::UpdateComponent::Request> request,
      std::shared_ptr<aic_flowstate_interface::srv::UpdateComponent::Response> response);

  TaskboardSpawnerLogic logic_;

  // Grpc Stubs
  std::unique_ptr<intrinsic_proto::assets::AssetDeploymentService::Stub> asset_deployment_stub_;
  std::unique_ptr<intrinsic_proto::assets::v1::InstalledAssets::Stub> installed_assets_stub_;
  std::unique_ptr<intrinsic_proto::world::ObjectWorldService::Stub> object_world_stub_;
  std::unique_ptr<google::longrunning::Operations::Stub> operations_stub_;

  rclcpp::Service<aic_flowstate_interface::srv::UpdateComponent>::SharedPtr update_service_;
};

}  // namespace flowstate
}  // namespace aic

#endif  // AIC_FLOWSTATE_SERVICES_TASKBOARD_SPAWNING_SERVICE_TASKBOARD_SPAWNER_NODE_H_

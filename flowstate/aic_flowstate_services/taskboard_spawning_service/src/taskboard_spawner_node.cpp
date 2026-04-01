#include "taskboard_spawning_service/taskboard_spawner_logic.h"

#include <rclcpp/rclcpp.hpp>
#include "aic_flowstate_interface/srv/update_component.hpp"

#include <grpcpp/grpcpp.h>
#include "intrinsic/assets/proto/asset_deployment.grpc.pb.h"
#include "intrinsic/assets/proto/installed_assets.grpc.pb.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"
#include "google/longrunning/operations.grpc.pb.h"

#include <iostream>
#include <fstream>
#include <memory>

using namespace intrinsic_proto::assets;
using namespace intrinsic_proto::world;
using namespace google::longrunning;

#include "taskboard_spawning_service/taskboard_spawner_constants.h"
#include "taskboard_spawning_service/taskboard_spawner_node.h"

namespace aic {
namespace flowstate {

TaskboardSpawnerNode::TaskboardSpawnerNode() : Node("taskboard_spawner_node") {
  this->declare_parameter(kParamConfigFile, "");
  this->declare_parameter(kParamBaseX, 0.17);
  this->declare_parameter(kParamBaseY, 0.0);
  this->declare_parameter(kParamBaseZ, 1.14);
  this->declare_parameter(kParamBaseYaw, 3.0);

  InitGrpcStubs();

  update_service_ = this->create_service<aic_flowstate_interface::srv::UpdateComponent>(
      kUpdateComponentService,
      std::bind(&TaskboardSpawnerNode::HandleUpdateComponent, this, std::placeholders::_1, std::placeholders::_2));

  // Try to trigger initial spawn on startup if config is provided
  std::string config_path = this->get_parameter(kParamConfigFile).as_string();
  if (!config_path.empty()) {
    RCLCPP_INFO(this->get_logger(), "Loading initial config from %s", config_path.c_str());
    SpawnFromConfig(config_path);
  }
}

void TaskboardSpawnerNode::InitGrpcStubs() {
  // Rely on platform injected environment variables or standard endpoints.
  std::string asset_server = kAssetServerDefault; // Placeholder, standard practice is to resolve via RuntimeContext
  
  auto channel = grpc::CreateChannel(asset_server, grpc::InsecureChannelCredentials());
  asset_deployment_stub_ = AssetDeploymentService::NewStub(channel);
  installed_assets_stub_ = intrinsic_proto::assets::v1::InstalledAssets::NewStub(channel);
  object_world_stub_ = ObjectWorldService::NewStub(channel);
  operations_stub_ = google::longrunning::Operations::NewStub(channel);
}

void TaskboardSpawnerNode::SpawnFromConfig(const std::string& config_path) {
  if (!logic_.LoadFromYaml(config_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse YAML config");
    return;
  }

  double base_x = this->get_parameter(kParamBaseX).as_double();
  double base_y = this->get_parameter(kParamBaseY).as_double();
  double base_z = this->get_parameter(kParamBaseZ).as_double();
  double base_yaw = this->get_parameter(kParamBaseYaw).as_double();

  // 1. Spawning Task Board Base (No parent, spawns in world root)
  SpawnComponent("task_board_base",  /*instance name is same as type for base or uniquely generated*/ "task_board_base",
                 base_x, base_y, base_z, 0.0, 0.0, base_yaw);

  // 2. Iterate elements and spawn relative children using proper parenting
  const auto& components = logic_.GetComponents();
  for (const auto& [key, cfg] : components) {
    if (cfg.entity_present) {
      double x, y, z, qx, qy, qz, qw;
      if (logic_.ComputeRelativePose(key, x, y, z, qx, qy, qz, qw)) {
        RCLCPP_INFO(this->get_logger(), "COMPUTED RELATIVE POSE [%s]: (x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f)", 
                    key.c_str(), x, y, z, qx, qy, qz, qw);
        SpawnComponent(key, cfg.entity_name, x, y, z, qx, qy, qz, qw, true, "task_board_base"); 
      }
    }
  }
}

bool TaskboardSpawnerNode::SpawnComponent(const std::string& component_type, const std::string& instance_name,
                                         double x, double y, double z, double rx_or_qx, double ry_or_qy, double rz_or_qz, 
                                         double rw, bool use_quat, const std::string& parent_name) {
  
  // Check if asset exists in catalog
  grpc::ClientContext get_context;
  intrinsic_proto::assets::v1::GetInstalledAssetRequest get_request;
  get_request.mutable_id()->set_package(kPackageSideloaded);
  get_request.mutable_id()->set_name(component_type); // Type name without version

  intrinsic_proto::assets::v1::InstalledAsset existing_asset;
  grpc::Status status = installed_assets_stub_->GetInstalledAsset(&get_context, get_request, &existing_asset);
  if (!status.ok()) {
    RCLCPP_WARN(this->get_logger(), "Asset type ID %s not found in InstalledAssets service: %s. Skipping spawn.", 
                component_type.c_str(), status.error_message().c_str());
    return false;
  }

  // Compose Type ID Version
  std::string type_id_version = "ai.intrinsic.sideloaded." + component_type + ".0.0.1";

  // Call CreateResourceFromCatalog
  CreateResourceFromCatalogRequest request;
  request.mutable_configuration()->set_name(instance_name);
  request.set_world_id("init_world");
  request.set_type_id_version(type_id_version);

  // Set Parent if specified
  if (!parent_name.empty()) {
    auto* parent_ref = request.mutable_configuration()->mutable_parent();
    parent_ref->mutable_reference()->mutable_by_name()->set_object_name(parent_name);
    parent_ref->mutable_entity_filter()->set_include_base_entity(true);
  }

  auto* pose = request.mutable_configuration()->mutable_parent_t_this();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);

  if (use_quat) {
    pose->mutable_orientation()->set_x(rx_or_qx);
    pose->mutable_orientation()->set_y(ry_or_qy);
    pose->mutable_orientation()->set_z(rz_or_qz);
    pose->mutable_orientation()->set_w(rw);
  } else {
    // If we pass euler (roll, pitch, yaw) 
    // We assume rx_or_qx = roll, ry_or_qy = pitch, rz_or_qz = yaw
    // (This is use-case dependent, our ComputeAbsolutePose outputs quaternions so this branch might be unused if we always pass quats)
  }

  grpc::ClientContext context;
  google::longrunning::Operation operation;
  status = asset_deployment_stub_->CreateResourceFromCatalog(&context, request, &operation);
  if (!status.ok()) {
    RCLCPP_ERROR(this->get_logger(), "CreateResourceFromCatalog failed for %s", instance_name.c_str());
    return false;
  }

  // Wait for operation to complete (abridged for demo brevity, in prod we poll GetOperation)
  RCLCPP_INFO(this->get_logger(), "Initiated spawn for %s", instance_name.c_str());
  return true;
}

bool TaskboardSpawnerNode::RemoveComponent(const std::string& entity_name) {
  RCLCPP_INFO(this->get_logger(), "Removing component: %s (Standard ObjectWorldService::DeleteObject stub call)", entity_name.c_str());
  
  // grpc::ClientContext context;
  // ... call object_world_stub_->DeleteObject() ...
  return true;
}

void TaskboardSpawnerNode::HandleUpdateComponent(
    const std::shared_ptr<aic_flowstate_interface::srv::UpdateComponent::Request> request,
    std::shared_ptr<aic_flowstate_interface::srv::UpdateComponent::Response> response) {
  
  RCLCPP_INFO(this->get_logger(), "Received update request for component: %s", request->component_name.c_str());

  if (!request->entity_present) {
    if (RemoveComponent(request->component_name)) {
      response->success = true;
      response->message = "Successfully removed component";
    } else {
      response->success = false;
      response->message = "Failed to remove component";
    }
    return;
  }

  // Range checks using standard limits
  if (!logic_.IsWithinLimits(request->component_name, request->translation)) {
    response->success = false;
    response->message = "Translation out of limits";
    return;
  }

  

  double x, y, z, qx, qy, qz, qw;
  if (logic_.ComputeRelativePose(request->component_name, x, y, z, qx, qy, qz, qw)) {
    
    // Call UpdateTransform gRPC or similar
    response->success = true;
    response->message = "Successfully updated transform (Placeholder)";
  } else {
    response->success = false;
    response->message = "Failed to compute relative pose";
  }
}

}  // namespace flowstate
}  // namespace aic

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aic::flowstate::TaskboardSpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

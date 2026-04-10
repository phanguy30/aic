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
#include <thread>
#include <chrono>
#include <cmath>

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
      std::string asset_name = GetMappedAssetName(key);

      double x, y, z, qx, qy, qz, qw;
      if (logic_.ComputeRelativePose(key, x, y, z, qx, qy, qz, qw)) {
        RCLCPP_INFO(this->get_logger(), "COMPUTED RELATIVE POSE [%s] -> Asset: %s: (x=%f, y=%f, z=%f, qx=%f, qy=%f, qz=%f, qw=%f)", 
                    key.c_str(), asset_name.c_str(), x, y, z, qx, qy, qz, qw);
        SpawnComponent(asset_name, key, x, y, z, qx, qy, qz, qw, true, "task_board_base"); 
      }
    }
  }
}

std::string TaskboardSpawnerNode::GetMappedAssetName(const std::string& key) {
  std::string asset_name = key;
  if (key.find("nic_rail") != std::string::npos) {
    asset_name = "nic_card_mount";
  } else if (key.find("sc_rail") != std::string::npos) {
    asset_name = "sc_port";
  }
  return asset_name;
}

bool TaskboardSpawnerNode::SpawnComponent(const std::string& component_type, const std::string& instance_name,
                                         double x, double y, double z, double rx_or_qx, double ry_or_qy, double rz_or_qz, 
                                         double rw, bool use_quat, const std::string& parent_name) {
  const char* token = std::getenv("FLOWSTATE_TOKEN");
  
  // Check if asset exists in catalog
  grpc::ClientContext get_context;
  if (token) {
    get_context.AddMetadata("authorization", "Bearer " + std::string(token));
  }
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
  const auto& id_version = existing_asset.metadata().id_version();
  std::string type_id_version = id_version.id().package() + "." + id_version.id().name() + "." + id_version.version();

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
    // If we pass euler (roll, pitch, yaw), assume rx_or_qx = roll, ry_or_qy = pitch, rz_or_qz = yaw
    double roll = rx_or_qx;
    double pitch = ry_or_qy;
    double yaw = rz_or_qz;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    pose->mutable_orientation()->set_w(cr * cp * cy + sr * sp * sy);
    pose->mutable_orientation()->set_x(sr * cp * cy - cr * sp * sy);
    pose->mutable_orientation()->set_y(cr * sp * cy + sr * cp * sy);
    pose->mutable_orientation()->set_z(cr * cp * sy - sr * sp * cy);
  }

  grpc::ClientContext context;
  if (token) {
    context.AddMetadata("authorization", "Bearer " + std::string(token));
  }
  google::longrunning::Operation operation;
  status = asset_deployment_stub_->CreateResourceFromCatalog(&context, request, &operation);
  if (!status.ok()) {
    RCLCPP_ERROR(this->get_logger(), "CreateResourceFromCatalog failed for %s", instance_name.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Initiated spawn for %s, waiting for completion...", instance_name.c_str());
  
  int retries = 0;
  const int kMaxOperationPollRetries = 15; // 30 seconds total
  while (!operation.done() && retries < kMaxOperationPollRetries) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    grpc::ClientContext op_context;
    if (token) {
      op_context.AddMetadata("authorization", "Bearer " + std::string(token));
    }
    google::longrunning::GetOperationRequest op_request;
    op_request.set_name(operation.name());
    status = operations_stub_->GetOperation(&op_context, op_request, &operation);
    if (!status.ok()) {
      RCLCPP_ERROR(this->get_logger(), "GetOperation RPC failed: %s", status.error_message().c_str());
      return false;
    }
    retries++;
  }

  if (!operation.done()) {
    RCLCPP_ERROR(this->get_logger(), "Timed out waiting for asset instantiation to complete for %s", instance_name.c_str());
    return false;
  }

  if (operation.has_error()) {
    RCLCPP_ERROR(this->get_logger(), "Asset instantiation failed for %s: %s", instance_name.c_str(), operation.error().message().c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully spawned %s", instance_name.c_str());
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

  // Update translation in logic before computing pose
  logic_.UpdateTranslation(request->component_name, request->translation);

  double x, y, z, qx, qy, qz, qw;
  if (logic_.ComputeRelativePose(request->component_name, x, y, z, qx, qy, qz, qw)) {
    auto it = logic_.GetComponents().find(request->component_name);
    if (it == logic_.GetComponents().end()) {
      response->success = false;
      response->message = "Component not found in config";
      return;
    }
    std::string entity_name = request->component_name;

    intrinsic_proto::world::UpdateTransformRequest update_req;
    update_req.set_world_id("init_world");

    auto* node_a = update_req.mutable_node_a();
    node_a->mutable_by_name()->mutable_object()->set_object_name("task_board_base");

    auto* node_b = update_req.mutable_node_b();
    node_b->mutable_by_name()->mutable_object()->set_object_name(entity_name);

    auto* pose = update_req.mutable_a_t_b();
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->mutable_position()->set_z(z);
    pose->mutable_orientation()->set_x(qx);
    pose->mutable_orientation()->set_y(qy);
    pose->mutable_orientation()->set_z(qz);
    pose->mutable_orientation()->set_w(qw);

    update_req.set_view(intrinsic_proto::world::ObjectView::BASIC);

    intrinsic_proto::world::UpdateTransformResponse update_resp;
    grpc::ClientContext context;
    const char* token = std::getenv("FLOWSTATE_TOKEN");
    if (token) {
      context.AddMetadata("authorization", "Bearer " + std::string(token));
    }
    grpc::Status status = object_world_stub_->UpdateTransform(&context, update_req, &update_resp);

    if (status.ok()) {
      response->success = true;
      response->message = "Successfully updated transform";
    } else {
      // If object not found, try to spawn it
      if (status.error_message().find("No object with name") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "Object %s not found, attempting to spawn...", request->component_name.c_str());
        std::string asset_name = GetMappedAssetName(request->component_name);
        
        if (SpawnComponent(asset_name, request->component_name, x, y, z, qx, qy, qz, qw, true, "task_board_base")) {
          response->success = true;
          response->message = "Successfully spawned component dynamically";
        } else {
          response->success = false;
          response->message = "Failed to dynamically spawn component";
        }
      } else {
        response->success = false;
        response->message = "Failed to update transform: " + status.error_message();
      }
    }
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

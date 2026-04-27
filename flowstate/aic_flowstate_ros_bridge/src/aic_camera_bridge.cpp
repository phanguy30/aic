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

#include "aic_camera_bridge.hpp"

#include <string>
#include <utility>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/platform/pubsub/zenoh_util/zenoh_config.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/connection_params.h"
#include "intrinsic/util/proto/get_text_proto.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/create_service.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/parameter.hpp"

using namespace std::chrono_literals;

namespace flowstate_ros_bridge {
constexpr const char* kDecimationParamName = "decimation";

///=============================================================================
void AicCameraBridge::declare_ros_parameters(
    ROSNodeInterfaces ros_node_interfaces) {
  const auto& param_interface =
      ros_node_interfaces
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

  param_interface->declare_parameter(kDecimationParamName,
                                     rclcpp::ParameterValue{1});
}

///=============================================================================
bool AicCameraBridge::initialize(
    ROSNodeInterfaces ros_node_interfaces,
    std::shared_ptr<Executive> /*executive_client*/,
    std::shared_ptr<World> /*world_client*/) {
  data_ = std::make_shared<Data>();

  data_->node_interfaces_ = std::move(ros_node_interfaces);

  const auto& param_interface =
      data_->node_interfaces_
          .get<rclcpp::node_interfaces::NodeParametersInterface>();

#if 0
  data_->server_address_ =
      param_interface->get_parameter(kServerAddressParamName)
          .get_value<std::string>();
#endif

  data_->pubsub_ = std::make_shared<intrinsic::PubSub>(
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>()
          ->get_name());

  // Create a Zenoh subscriber on the robot_state pubsub topic
  auto image_sub = data_->pubsub_->CreateSubscription<sensor_msgs::msg::pb::jazzy::Image>(
      "cameras/*/image", intrinsic::TopicConfig(),
      [this](const sensor_msgs::msg::pb::jazzy::Image& image_msg) {
        this->ImageCallback(image_msg);
      });
  if (!image_sub.ok()) {
    LOG(ERROR) << "Unable to create Flowstate image subscription: "
               << image_sub.status();
    return false;
  }
  LOG(INFO) << "Subscribed to Flowstate camera image topics";
  data_->image_sub_ = std::make_shared<intrinsic::Subscription>(std::move(*image_sub));

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
      topics_interface =
          data_->node_interfaces_
              .get<rclcpp::node_interfaces::NodeTopicsInterface>();
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface =
      data_->node_interfaces_.get<rclcpp::node_interfaces::NodeBaseInterface>();
  auto clock = data_->node_interfaces_
                   .get<rclcpp::node_interfaces::NodeClockInterface>()
                   ->get_clock();

  // Reliable QoS subscriptions for motion commands.
  rclcpp::QoS image_pub_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();

  data_->center_image_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(
      topics_interface, "/center_camera/image", image_pub_qos);

  LOG(INFO) << "Initialized AicCameraBridge.";

  return true;
}

///=============================================================================
void AicCameraBridge::ImageCallback(
    const sensor_msgs::msg::pb::jazzy::Image& image) {
  LOG(INFO) << "ImageCallback() image.header.frame_id: " << image.header().frame_id();

  const absl::Time start_time = absl::Now();
  sensor_msgs::msg::Image ros_image;

  // Populate header
  ros_image.header.stamp.sec = image.header().stamp().sec();
  ros_image.header.stamp.nanosec = image.header().stamp().nanosec();
  ros_image.header.frame_id = image.header().frame_id();

  // Populate image metadata
  ros_image.height = image.height();
  ros_image.width = image.width();
  ros_image.encoding = image.encoding();
  ros_image.is_bigendian = image.is_bigendian();
  ros_image.step = image.step();

  // Populate image data
  ros_image.data.assign(image.data().begin(), image.data().end());

  const absl::Duration duration = absl::Now() - start_time;

  // Route based on frame_id
  const std::string& frame_id = ros_image.header.frame_id;
  if (frame_id.find("left_camera") != std::string::npos) {
    data_->left_image_pub_->publish(ros_image);
  } else if (frame_id.find("center_camera") != std::string::npos) {
    static int center_callback_count = 0;
    center_callback_count++;
    if (center_callback_count % 100 == 0) {
      LOG(INFO) << absl::StrFormat(
          "TIME TO POPULATE ROS IMAGE FOR CENTER CAMERA: %.6F SECONDS",
          absl::ToDoubleSeconds(duration));
    }
    data_->center_image_pub_->publish(ros_image);
  } else if (frame_id.find("right_camera") != std::string::npos) {
    data_->right_image_pub_->publish(ros_image);
  } else {
    LOG(WARNING) << "Unknown camera frame_id: " << frame_id;
  }
}

///=============================================================================
AicCameraBridge::Data::Data() {}

///=============================================================================
AicCameraBridge::Data::~Data() {
  pubsub_.reset();
  image_sub_.reset();
  left_image_pub_.reset();
  center_image_pub_.reset();
  right_image_pub_.reset();
}

}  // namespace flowstate_ros_bridge

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(flowstate_ros_bridge::AicCameraBridge,
                       flowstate_ros_bridge::BridgeInterface)

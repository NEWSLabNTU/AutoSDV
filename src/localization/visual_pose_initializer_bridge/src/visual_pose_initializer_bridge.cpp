// Copyright 2025 AutoSDV
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "autoware_internal_localization_msgs/srv/initialize_localization.hpp"

using namespace std::chrono_literals;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using InitializeLocalization = autoware_internal_localization_msgs::srv::InitializeLocalization;

class VisualPoseInitializerBridge : public rclcpp::Node
{
public:
  enum class State {
    WAITING,       // Waiting for first valid pose from cuVGL
    INITIALIZING,  // Calling Autoware initialize service
    INITIALIZED,   // Successfully initialized
    ERROR          // Error state
  };

  VisualPoseInitializerBridge()
  : Node("visual_pose_initializer_bridge"),
    state_(State::WAITING)
  {
    // Declare parameters
    this->declare_parameter("auto_initialize", true);
    this->declare_parameter("initialization_method", 1);  // DIRECT = 1
    this->declare_parameter("reinitialize_on_trigger", true);
    this->declare_parameter("service_timeout_sec", 5.0);
    this->declare_parameter("pose_topic", "/visual_localization/pose");
    this->declare_parameter("initialize_service", "/localization/initialize");

    // Get parameters
    auto_initialize_ = this->get_parameter("auto_initialize").as_bool();
    initialization_method_ = this->get_parameter("initialization_method").as_int();
    reinitialize_on_trigger_ = this->get_parameter("reinitialize_on_trigger").as_bool();
    service_timeout_ = this->get_parameter("service_timeout_sec").as_double();
    pose_topic_ = this->get_parameter("pose_topic").as_string();
    initialize_service_ = this->get_parameter("initialize_service").as_string();

    // Create subscriber for cuVGL pose
    pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
      pose_topic_, 10,
      std::bind(&VisualPoseInitializerBridge::pose_callback, this, std::placeholders::_1));

    // Create service client for Autoware initialization
    initialize_client_ = this->create_client<InitializeLocalization>(initialize_service_);

    // Create trigger service for manual re-initialization
    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/trigger_initialization",
      std::bind(&VisualPoseInitializerBridge::trigger_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Visual Pose Initializer Bridge started");
    RCLCPP_INFO(this->get_logger(), "  Pose topic: %s", pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Initialize service: %s", initialize_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Auto initialize: %s", auto_initialize_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Method: %s", initialization_method_ == 1 ? "DIRECT" : "AUTO");
  }

private:
  void pose_callback(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_pose_ = msg;

    // Only auto-initialize on first pose if enabled
    if (auto_initialize_ && state_ == State::WAITING) {
      RCLCPP_INFO(this->get_logger(), "Received first pose from cuVGL, initializing...");
      call_initialize_service(msg);
    }
  }

  void trigger_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!last_pose_) {
      response->success = false;
      response->message = "No pose received from cuVGL yet";
      RCLCPP_WARN(this->get_logger(), "Trigger failed: %s", response->message.c_str());
      return;
    }

    if (!reinitialize_on_trigger_ && state_ == State::INITIALIZED) {
      response->success = false;
      response->message = "Already initialized and reinitialize_on_trigger is false";
      RCLCPP_WARN(this->get_logger(), "Trigger failed: %s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Manual initialization triggered");
    bool result = call_initialize_service(last_pose_);
    response->success = result;
    response->message = result ? "Initialization successful" : "Initialization failed";
  }

  bool call_initialize_service(const PoseWithCovarianceStamped::SharedPtr & pose)
  {
    if (!initialize_client_->wait_for_service(std::chrono::duration<double>(service_timeout_))) {
      RCLCPP_ERROR(this->get_logger(), "Initialize service not available");
      state_ = State::ERROR;
      return false;
    }

    state_ = State::INITIALIZING;

    auto request = std::make_shared<InitializeLocalization::Request>();
    request->pose_with_covariance.push_back(*pose);
    request->method = static_cast<uint8_t>(initialization_method_);

    RCLCPP_INFO(this->get_logger(), "Calling initialize service with pose (%.2f, %.2f, %.2f)",
      pose->pose.pose.position.x,
      pose->pose.pose.position.y,
      pose->pose.pose.position.z);

    auto future = initialize_client_->async_send_request(request);

    // Wait for the result
    if (future.wait_for(std::chrono::duration<double>(service_timeout_)) == std::future_status::ready) {
      auto response = future.get();
      if (response->status.success) {
        state_ = State::INITIALIZED;
        RCLCPP_INFO(this->get_logger(), "Pose initialization successful");
        return true;
      } else {
        state_ = State::ERROR;
        RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s (code: %d)",
          response->status.message.c_str(), response->status.code);
        return false;
      }
    } else {
      state_ = State::ERROR;
      RCLCPP_ERROR(this->get_logger(), "Initialize service call timed out");
      return false;
    }
  }

  // State
  State state_;
  PoseWithCovarianceStamped::SharedPtr last_pose_;

  // Parameters
  bool auto_initialize_;
  int initialization_method_;
  bool reinitialize_on_trigger_;
  double service_timeout_;
  std::string pose_topic_;
  std::string initialize_service_;

  // ROS interfaces
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Client<InitializeLocalization>::SharedPtr initialize_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualPoseInitializerBridge>());
  rclcpp::shutdown();
  return 0;
}

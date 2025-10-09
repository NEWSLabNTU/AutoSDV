// Copyright 2025 AutoSDV Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

/**
 * @brief Bridge node to convert nav_msgs::Odometry to geometry_msgs::PoseWithCovarianceStamped
 *
 * This node subscribes to odometry messages (typically from Isaac ROS Visual SLAM) and
 * republishes the pose information as PoseWithCovarianceStamped for consumption by
 * Autoware's localization module (EKF localizer).
 */
class OdometryToPoseBridge : public rclcpp::Node
{
public:
  OdometryToPoseBridge()
  : Node("odometry_to_pose_bridge")
  {
    // Declare parameters
    this->declare_parameter("queue_size", 10);

    int queue_size = this->get_parameter("queue_size").as_int();

    // Create subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "input/odometry",
      queue_size,
      std::bind(&OdometryToPoseBridge::odometry_callback, this, std::placeholders::_1)
    );

    // Create publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "output/pose_with_covariance",
      queue_size
    );

    RCLCPP_INFO(this->get_logger(), "Odometry to PoseWithCovariance bridge started");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: input/odometry");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: output/pose_with_covariance");
  }

private:
  /**
   * @brief Callback for odometry messages
   *
   * Converts nav_msgs::Odometry to geometry_msgs::PoseWithCovarianceStamped by:
   * 1. Copying the header (frame_id and timestamp)
   * 2. Copying the pose (position and orientation)
   * 3. Copying the 6x6 pose covariance matrix
   */
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

    // Copy header (frame_id and timestamp)
    pose_msg.header = msg->header;

    // Copy pose (position and orientation)
    pose_msg.pose.pose = msg->pose.pose;

    // Copy covariance (6x6 pose covariance matrix)
    // nav_msgs::Odometry has a 6x6 pose covariance matrix
    // geometry_msgs::PoseWithCovarianceStamped also has a 6x6 covariance matrix
    for (size_t i = 0; i < 36; ++i) {
      pose_msg.pose.covariance[i] = msg->pose.covariance[i];
    }

    // Publish the converted message
    pose_pub_->publish(pose_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<OdometryToPoseBridge>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("odometry_to_pose_bridge"), "Exception: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

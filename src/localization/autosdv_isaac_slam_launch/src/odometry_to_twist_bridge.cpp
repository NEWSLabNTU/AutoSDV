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
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

/**
 * @brief Bridge node to convert nav_msgs::Odometry to geometry_msgs::TwistWithCovarianceStamped
 *
 * This node subscribes to odometry messages (typically from Isaac ROS Visual SLAM) and
 * republishes the twist (velocity) information as TwistWithCovarianceStamped for consumption by
 * Autoware's localization module (EKF localizer).
 *
 * In the AR tag + VSLAM fusion architecture:
 * - AR tag localizer provides global pose (periodic corrections)
 * - Isaac VSLAM provides local twist/velocity (continuous high-rate tracking)
 * - EKF fuses both for drift-free localization
 */
class OdometryToTwistBridge : public rclcpp::Node
{
public:
  OdometryToTwistBridge()
  : Node("odometry_to_twist_bridge")
  {
    // Declare parameters
    this->declare_parameter("queue_size", 10);
    this->declare_parameter("output_frame_id", "base_link");

    int queue_size = this->get_parameter("queue_size").as_int();
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();

    // Create subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "input/odometry",
      queue_size,
      std::bind(&OdometryToTwistBridge::odometry_callback, this, std::placeholders::_1)
    );

    // Create publisher
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "output/twist_with_covariance",
      queue_size
    );

    RCLCPP_INFO(this->get_logger(), "Odometry to TwistWithCovariance bridge started");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: input/odometry");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: output/twist_with_covariance");
    RCLCPP_INFO(this->get_logger(), "  Output frame_id: %s", output_frame_id_.c_str());
  }

private:
  /**
   * @brief Callback for odometry messages
   *
   * Converts nav_msgs::Odometry to geometry_msgs::TwistWithCovarianceStamped by:
   * 1. Copying the timestamp
   * 2. Setting the frame_id to base_link (vehicle frame for twist)
   * 3. Copying the twist (linear and angular velocities)
   * 4. Copying the 6x6 twist covariance matrix
   */
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto twist_msg = geometry_msgs::msg::TwistWithCovarianceStamped();

    // Copy timestamp
    twist_msg.header.stamp = msg->header.stamp;

    // Set frame_id to base_link (twist is in vehicle body frame)
    // Note: Different from pose which is typically in map/odom frame
    twist_msg.header.frame_id = output_frame_id_;

    // Copy twist (linear and angular velocities)
    twist_msg.twist.twist = msg->twist.twist;

    // Copy covariance (6x6 twist covariance matrix)
    // nav_msgs::Odometry has a 6x6 twist covariance matrix
    // geometry_msgs::TwistWithCovarianceStamped also has a 6x6 covariance matrix
    // Format: [vx, vy, vz, wx, wy, wz] variances and covariances
    for (size_t i = 0; i < 36; ++i) {
      twist_msg.twist.covariance[i] = msg->twist.covariance[i];
    }

    // Publish the converted message
    twist_pub_->publish(twist_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  std::string output_frame_id_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<OdometryToTwistBridge>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("odometry_to_twist_bridge"), "Exception: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

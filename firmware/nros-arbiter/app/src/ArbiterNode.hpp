/// @file ArbiterNode.hpp
/// @brief The arbiter's ROS face: what mode it is in, for the Orin and for logs.
///
/// Publish-only. Nothing the Orin sends over ROS can change who drives; the
/// mode is decided by the switch, the receiver and the PCA9685 heartbeat
/// (docs/design/rc-override-arbiter.md), and this node only reports it.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs.hpp"

namespace autosdv_arbiter {

class ArbiterNode {
    ::rclcpp::Publisher<std_msgs::msg::String> pub_;
    ::rclcpp::Timer timer_;

    void on_tick();

  public:
    ::rclcpp::Result configure(::rclcpp::Node& node);
};

} // namespace autosdv_arbiter

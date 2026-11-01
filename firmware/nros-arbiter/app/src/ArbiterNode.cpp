/// @file ArbiterNode.cpp
#include "ArbiterNode.hpp"

#include "arbiter.h"
#include "control.h"

namespace autosdv_arbiter {

namespace {
// 10 Hz: a mode change reaches the Orin within 100 ms, and the AUTO line
// (D10) is the fast path anyway.
constexpr uint32_t kPeriodMs = 100;
} // namespace

void ArbiterNode::on_tick() {
    std_msgs::msg::String m;
    m.data = arbiter::mode_name(control::mode());
    (void)pub_.publish(m);
}

::rclcpp::Result ArbiterNode::configure(::rclcpp::Node& node) {
    ::rclcpp::Result r = node.create_publisher(pub_, "/vehicle/arbiter/mode");
    if (!r.ok()) return r;
    return node.create_wall_timer<ArbiterNode, &ArbiterNode::on_tick>(timer_, kPeriodMs, this);
}

} // namespace autosdv_arbiter

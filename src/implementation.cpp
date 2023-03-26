/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_switch/implementation.hpp"

#include <functional>

#include "rclcpp/qos.hpp"
#include "remote_switch/config.hpp"
#include "remote_switch/srv/switch.hpp"

namespace remote_switch {

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {
  node->declare_parameter("switch_channels", 1);
  node->get_parameter("switch_channels", channels_num_);

  // TODO(clairbee): consider initializing the channels ahead of time
  // for (int i = 0; i < channels_num_.as_int(); i++) {
  //   channels_.emplace(
  //       std::make_shared<ChannelState>(node, get_prefix_(), i));
  // }

  srv_switch = node_->create_service<srv::Switch>(
      get_prefix_() + SWITCH_SERVICE_SWITCH,
      std::bind(&Implementation::switch_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
}

Implementation::ChannelState::ChannelState(rclcpp::Node *node,
                                           const std::string &interface_prefix,
                                           int channel) {
  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

  last_changed = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix + "/channel" + std::to_string(channel) + "/last_changed",
      qos);
  last_on = node->create_publisher<std_msgs::msg::Bool>(
      interface_prefix + "/channel" + std::to_string(channel) + "/last_on",
      qos);
}

void Implementation::switch_single_cmd(bool on) {
  (void)on;
  // TODO(clairbee): refactor API to eliminate the need for this method
  //                 in the implementation object
}

void Implementation::switch_cmd(uint16_t channel, bool on) {
  std::shared_ptr<srv::Switch::Request> request;
  std::shared_ptr<srv::Switch::Response> response;
  request->channel = channel;
  request->on = on;

  switch_handler_(request, response);
}

void Implementation::switch_handler_(
    const std::shared_ptr<srv::Switch::Request> request,
    std::shared_ptr<srv::Switch::Response> response) {
  if (/* |suppress the warning| request->channel < 0 ||*/ request->channel >=
      channels_num_.as_int()) {
    response->exception_code = 1;
    return;
  }

  switch_handler_real_(request, response);

  auto now = rclcpp::Time().nanoseconds();
  std_msgs::msg::UInt64 msg_now;
  msg_now.data = now;

  std_msgs::msg::Bool msg_on;
  msg_on.data = request->on;

  auto prefix = get_prefix_();
  channels_lock_.lock();
  if (channels_.find(request->channel) == channels_.end()) {
    channels_.emplace(request->channel, std::make_shared<ChannelState>(
                                            node_, prefix, request->channel));
  }
  channels_[request->channel]->last_changed->publish(msg_now);
  channels_[request->channel]->last_on->publish(msg_on);
  channels_lock_.unlock();
}

}  // namespace remote_switch

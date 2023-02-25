/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "switch_interface/implementation.hpp"

#include <functional>

#include "switch_interface/config.hpp"
#include "switch_interface/srv/switch.hpp"

namespace switch_interface {

Implementation::Implementation(rclcpp::Node *node) : Interface(node) {
  node->declare_parameter("switch_channels", 1);
  node->get_parameter("switch_channels", channels_num_);

  // TODO(clairbee): consider initializing the channels ahead of time
  // for (int i = 0; i < channels_num_.as_int(); i++) {
  //   channels_.emplace(
  //       std::make_shared<ChannelState>(node, interface_prefix_, i));
  // }

  srv_switch =
      node_->create_service<switch_interface::srv::Switch>(
          interface_prefix_.as_string() + SWITCH_SERVICE_SWITCH,
          std::bind(&Implementation::switch_handler_, this,
                    std::placeholders::_1, std::placeholders::_2),
          ::rmw_qos_profile_default, callback_group_);
}

Implementation::ChannelState::ChannelState(rclcpp::Node *node,
                                      const std::string &interface_prefix,
                                      int channel) {
  last_changed = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix + "/channel" + std::to_string(channel) + "/last_changed",
      10);
  last_on = node->create_publisher<std_msgs::msg::Bool>(
      interface_prefix + "/channel" + std::to_string(channel) + "/last_on", 10);
}

void Implementation::switch_single_cmd(bool on) {
  (void)on;
  // TODO(clairbee): refactor API to eliminate the need for this method
  //                 in the implementation object
}

void Implementation::switch_cmd(uint16_t channel, bool on) {
  std::shared_ptr<switch_interface::srv::Switch::Request> request;
  std::shared_ptr<switch_interface::srv::Switch::Response> response;
  request->channel = channel;
  request->on = on;

  switch_handler_(request, response);
}

void Implementation::switch_handler_(
    const std::shared_ptr<switch_interface::srv::Switch::Request> request,
    std::shared_ptr<switch_interface::srv::Switch::Response> response) {
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

  auto prefix = interface_prefix_.as_string();
  channels_lock_.lock();
  if (channels_.find(request->channel) == channels_.end()) {
    channels_.emplace(
        request->channel,
        std::make_shared<ChannelState>(node_, prefix,
                                       request->channel));
  }
  channels_[request->channel]->last_changed->publish(msg_now);
  channels_[request->channel]->last_on->publish(msg_on);
  channels_lock_.unlock();
}

}  // namespace switch_interface
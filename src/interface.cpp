/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "switch_interface/interface.hpp"

#include <functional>

#include "switch_interface/config.hpp"
#include "switch_interface/srv/switch.hpp"

namespace switch_interface {

Interface::Interface(rclcpp::Node *node) : node_{node} {
  node->declare_parameter("switch_prefix", "/switch/default");
  node->get_parameter("switch_prefix", interface_prefix_);
  node->declare_parameter("switch_channels", 1);
  node->get_parameter("switch_channels", channels_num_);

  // TODO(clairbee): consider initializing the channels ahead of time
  // for (int i = 0; i < channels_num_.as_int(); i++) {
  //   channels_.emplace(
  //       std::make_shared<ChannelState>(node, interface_prefix_, i));
  // }
}

Interface::ChannelState::ChannelState(rclcpp::Node *node,
                                      const std::string &interface_prefix,
                                      int channel) {
  last_changed = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix + "/channel" + std::to_string(channel) + "/last_changed",
      10);
  last_on = node->create_publisher<std_msgs::msg::Bool>(
      interface_prefix + "/channel" + std::to_string(channel) + "/last_on", 10);
}

void Interface::switch_handler_(
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

  channels_lock_.lock();
  if (channels_.find(request->channel) == channels_.end()) {
    channels_.emplace(
        request->channel,
        std::make_shared<ChannelState>(node_, interface_prefix_.as_string(),
                                       request->channel));
  }
  channels_[request->channel]->last_changed->publish(msg_now);
  channels_[request->channel]->last_on->publish(msg_on);
  channels_lock_.unlock();
}

}  // namespace switch_interface
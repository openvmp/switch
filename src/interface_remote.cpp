/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_switch/interface_remote.hpp"

#include <functional>

#include "remote_switch/config.hpp"

namespace remote_switch {

RemoteInterface::RemoteInterface(rclcpp::Node *node) : Interface(node) {
  node->declare_parameter("switch_channel", 0);
  node->get_parameter("switch_channel", channel_num_);

  clnt_switch = node->create_client<srv::Switch>(
      get_prefix_() + SWITCH_SERVICE_SWITCH, ::rmw_qos_profile_default,
      callback_group_);
}

void RemoteInterface::switch_single_cmd(bool on) {
  std::shared_ptr<srv::Switch::Request> request;
  std::shared_ptr<srv::Switch::Response> response;
  request->channel = channel_num_.as_int();
  request->on = on;

  auto f = clnt_switch->async_send_request(request);
  f.wait();
  *response = *f.get();
}

void RemoteInterface::switch_cmd(uint16_t channel, bool on) {
  std::shared_ptr<srv::Switch::Request> request;
  std::shared_ptr<srv::Switch::Response> response;
  request->channel = channel;
  request->on = on;

  auto f = clnt_switch->async_send_request(request);
  f.wait();
  *response = *f.get();
}

}  // namespace remote_switch
/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_switch/interface.hpp"

#include <functional>

#include "remote_switch/config.hpp"
#include "remote_switch/srv/switch.hpp"

namespace remote_switch {

Interface::Interface(rclcpp::Node *node) : node_{node} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("switch_prefix",
                          "/switch/" + std::string(node_->get_name()));
  node->get_parameter("switch_prefix", interface_prefix_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace remote_switch

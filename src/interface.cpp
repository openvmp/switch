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
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("switch_prefix", "/switch/" + std::string(node_->get_name()));
  node->get_parameter("switch_prefix", interface_prefix_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() >0 && prefix[prefix.length()-1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace switch_interface

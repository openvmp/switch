/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SWITCH_INTERFACE_H
#define OPENVMP_SWITCH_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "remote_switch/srv/switch.hpp"
#include "std_msgs/msg/u_int16.hpp"

#define SWITCH_SERVICE_SWITCH "/switch"

namespace remote_switch {

class Interface {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface() {}

  virtual void switch_single_cmd(bool on) = 0;
  virtual void switch_cmd(uint16_t channel, bool on) = 0;

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string get_prefix_();

 private:
  rclcpp::Parameter interface_prefix_;
};

}  // namespace remote_switch

#endif  // OPENVMP_SWITCH_INTERFACE_H

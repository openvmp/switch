/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SWITCH_IMPLEMENTATION_H
#define OPENVMP_SWITCH_IMPLEMENTATION_H

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "switch_interface/interface.hpp"
#include "switch_interface/srv/switch.hpp"

namespace switch_interface {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node);
  virtual ~Implementation() {}

 protected:
  rclcpp::Parameter channels_num_;

  virtual void switch_handler_real_(
      const std::shared_ptr<switch_interface::srv::Switch::Request> request,
      std::shared_ptr<switch_interface::srv::Switch::Response> response) = 0;

  class ChannelState {
   public:
    ChannelState(rclcpp::Node *node, const std::string &prefix, int channel);

    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr last_changed;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr last_on;
  };
  std::map<int, std::shared_ptr<ChannelState>> channels_;

 private:
  std::mutex channels_lock_;

  virtual void switch_cmd(uint16_t channel, bool on) override;
  virtual void switch_handler_(
      const std::shared_ptr<switch_interface::srv::Switch::Request> request,
      std::shared_ptr<switch_interface::srv::Switch::Response> response)
      override;
};

}  // namespace switch_interface

#endif  // OPENVMP_SWITCH_IMPLEMENTATION_H
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
#include "remote_switch/interface.hpp"
#include "remote_switch/srv/switch.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace remote_switch {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node);
  virtual ~Implementation() {}

 protected:
  rclcpp::Parameter channels_num_;

  // TODO(clairbee): avoid composite types here
  virtual void switch_handler_real_(
      const std::shared_ptr<srv::Switch::Request> request,
      std::shared_ptr<srv::Switch::Response> response) = 0;

  class ChannelState {
   public:
    ChannelState(rclcpp::Node *node, const std::string &prefix, int channel);

    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr last_changed;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr last_on;
  };
  std::map<int, std::shared_ptr<ChannelState>> channels_;

 private:
  std::mutex channels_lock_;

  rclcpp::Service<srv::Switch>::SharedPtr srv_switch;

  virtual void switch_single_cmd(bool on) override;
  virtual void switch_cmd(uint16_t channel, bool on) override;
  void switch_handler_(const std::shared_ptr<srv::Switch::Request> request,
                       std::shared_ptr<srv::Switch::Response> response);
};

}  // namespace remote_switch

#endif  // OPENVMP_SWITCH_IMPLEMENTATION_H
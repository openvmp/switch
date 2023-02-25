/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2023-02-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SWITCH_INTERFACE_REMOTE_H
#define OPENVMP_SWITCH_INTERFACE_REMOTE_H

#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "switch_interface/interface.hpp"

namespace switch {

  class RemoteInterface : public Interface {
   public:
    RemoteInterface(rclcpp::Node *node);
    virtual ~RemoteInterface() {}

    virtual void switch_cmd(uint16_t channel, bool on) override;

   private:
    rclcpp::Client<switch_interface::srv::Switch>::SharedPtr clnt_switch;
  };

}  // namespace switch

#endif  // OPENVMP_MODBUS_INTERFACE_REMOTE_H

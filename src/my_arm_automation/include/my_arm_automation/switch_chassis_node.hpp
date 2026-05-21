#ifndef MY_ARM_AUTOMATION_SWITCH_CHASSIS_NODE_HPP
#define MY_ARM_AUTOMATION_SWITCH_CHASSIS_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

class SwitchChassis : public BT::SyncActionNode {
public:
  SwitchChassis(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

#endif
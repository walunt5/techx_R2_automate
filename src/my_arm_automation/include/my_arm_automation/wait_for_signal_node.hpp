#ifndef MY_ARM_AUTOMATION_WAIT_FOR_SIGNAL_NODE_HPP
#define MY_ARM_AUTOMATION_WAIT_FOR_SIGNAL_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 
#include <string>

class WaitForSignal : public BT::StatefulActionNode {
public:
  WaitForSignal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  bool signal_received_;
};

#endif // MY_ARM_AUTOMATION_WAIT_FOR_SIGNAL_NODE_HPP
#include "my_arm_automation/control_gripper_node.hpp"

ControlGripper::ControlGripper(const std::string& name, const BT::NodeConfig& config,
                               std::shared_ptr<rclcpp::Node> node,
                               const std::string& topic_name)
  : BT::SyncActionNode(name, config), topic_name_(topic_name)
{
    pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_name_, 10);
}

BT::PortsList ControlGripper::providedPorts() {
   // 接收 1.0 (闭合) 或 0.0 (张开)
   return { BT::InputPort<double>("command") }; 
}

BT::NodeStatus ControlGripper::tick() {
  double cmd = 0.0;
  if (!getInput("command", cmd)) {
      RCLCPP_ERROR(rclcpp::get_logger("BT_Gripper"), "Missing command input");
      return BT::NodeStatus::FAILURE;
  }

  // 组装并发布浮点数指令
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(cmd);
  pub_->publish(msg);

  RCLCPP_INFO(rclcpp::get_logger("BT_Gripper"), 
              "瞬间下发夹爪战略指令 -> %.1f (1.0=闭合, 0.0=张开)", cmd);

  return BT::NodeStatus::SUCCESS; 
}
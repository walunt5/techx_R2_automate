#ifndef MY_ARM_AUTOMATION_CONTROL_GRIPPER_NODE_HPP
#define MY_ARM_AUTOMATION_CONTROL_GRIPPER_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <memory> // 添加 memory 头文件

class ControlGripper : public BT::SyncActionNode {
public:
  ControlGripper(const std::string& name, const BT::NodeConfig& config,
                 std::shared_ptr<rclcpp::Node> node, 
                 const std::string& topic_name); 
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  std::string topic_name_; 
};

#endif // MY_ARM_AUTOMATION_CONTROL_GRIPPER_NODE_HPP
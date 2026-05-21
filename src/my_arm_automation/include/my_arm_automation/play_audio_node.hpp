#ifndef MY_ARM_AUTOMATION_PLAY_AUDIO_NODE_HPP
#define MY_ARM_AUTOMATION_PLAY_AUDIO_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PlayAudio : public BT::SyncActionNode {
public:
  // 构造函数需要传入 ROS2 Node 句柄来创建发布者
  PlayAudio(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audio_pub_;
};

#endif
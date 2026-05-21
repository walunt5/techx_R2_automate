#include "my_arm_automation/play_audio_node.hpp"

PlayAudio::PlayAudio(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config) 
{
  audio_pub_ = node->create_publisher<std_msgs::msg::String>("/audio_play", 10);
}

BT::PortsList PlayAudio::providedPorts() {
  // 定义一个输入端口，这样你可以在 XML 里指定播哪个文件
  return { BT::InputPort<std::string>("audio_file") };
}

BT::NodeStatus PlayAudio::tick() {
  std::string file;
  if (!getInput("audio_file", file)) {
    RCLCPP_ERROR(rclcpp::get_logger("BT_Audio"), "未指定音频文件！");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::String msg;
  msg.data = file;
  audio_pub_->publish(msg);
  
  RCLCPP_INFO(rclcpp::get_logger("BT_Audio"), "指令下发：播放音频 [%s]", file.c_str());
  return BT::NodeStatus::SUCCESS;
}
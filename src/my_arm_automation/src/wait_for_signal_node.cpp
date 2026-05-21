#include "my_arm_automation/wait_for_signal_node.hpp"

WaitForSignal::WaitForSignal(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), signal_received_(false) 
{
  // 订阅新的视觉/指令字符串话题
  sub_ = node->create_subscription<std_msgs::msg::String>(
    "/vision/signals", 10, 
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::string expected_signal;
      // 从 XML 中读取当前期待的字符串 (比如 "ASSEMBLY_DONE")
      if (this->getInput("expected", expected_signal)) {
          // 如果收到的字符串和期待的一字不差，就触发放行标志
          if (msg->data == expected_signal) {
              this->signal_received_ = true;
          }
      }
    });
}

BT::PortsList WaitForSignal::providedPorts() { 
  // 👈 核心修复：声明自己需要 XML 传入 expected 端口
  return { BT::InputPort<std::string>("expected") }; 
}

BT::NodeStatus WaitForSignal::onStart() {
    // 👈 核心修改：进入节点时，立刻清空旧信号
    // 这样它就必须等待“从这一刻起”之后发送的新信号
    signal_received_ = false; 
    
    std::string expected_signal;
    getInput("expected", expected_signal);
    RCLCPP_INFO(rclcpp::get_logger("BT_Wait"), "开始等待新信号 [%s]...", expected_signal.c_str());
    return BT::NodeStatus::RUNNING; 
}

BT::NodeStatus WaitForSignal::onRunning() {
  if (signal_received_) {
    std::string expected_signal;
    getInput("expected", expected_signal);
    RCLCPP_INFO(rclcpp::get_logger("BT_Wait"), "WaitForSignal: 成功接收并匹配信号 [%s]！放行！", expected_signal.c_str());
    signal_received_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING; 
}

void WaitForSignal::onHalted() {
  RCLCPP_WARN(rclcpp::get_logger("BT_Wait"), "WaitForSignal: 等待动作被紧急打断！");
}
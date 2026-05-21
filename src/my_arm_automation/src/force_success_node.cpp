#include "my_arm_automation/force_success_node.hpp"

ForceSuccess::ForceSuccess(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList ForceSuccess::providedPorts() {
    return {};
}

BT::NodeStatus ForceSuccess::tick() {
    // 故意使用 WARN 级别（黄色字体），在赛场上满屏白字的终端里一眼就能看到
    RCLCPP_WARN(rclcpp::get_logger("BT_Core"), "[%s] 触发强制放行！正在掩盖局部错误，推进 R2 主流程！", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
}
#include "my_arm_automation/force_failure_node.hpp"

ForceFailure::ForceFailure(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList ForceFailure::providedPorts() {
    return {};
}

BT::NodeStatus ForceFailure::tick() {
    // 故意使用 ERROR 级别（红色字体）
    RCLCPP_ERROR(rclcpp::get_logger("BT_Core"), "[%s] 主动抛出失败！触发上一层 Fallback 残局收拾逻辑！", this->name().c_str());
    return BT::NodeStatus::FAILURE;
}
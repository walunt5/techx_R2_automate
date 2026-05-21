#include "my_arm_automation/wait_gripper_status_node.hpp"
#include <cmath>

WaitGripperStatus::WaitGripperStatus(const std::string& name, const BT::NodeConfig& config, 
                                     std::shared_ptr<rclcpp::Node> node,
                                     const std::string& topic_name)
    : BT::StatefulActionNode(name, config), node_(node)
{
    status_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        topic_name, 10,  // <--- 动态话题名
        std::bind(&WaitGripperStatus::statusCallback, this, std::placeholders::_1)
    );
}

BT::PortsList WaitGripperStatus::providedPorts() {
    return { 
        BT::InputPort<double>("expected_status"),
        BT::InputPort<double>("timeout")         
    };
}

void WaitGripperStatus::statusCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    latest_gripper_status_ = msg->data;
}

BT::NodeStatus WaitGripperStatus::onStart() {
    start_time_ = std::chrono::steady_clock::now();
    latest_gripper_status_ = 9.9; // 初始为未知状态
    
    double expected_status;
    if (!getInput("expected_status", expected_status)) {
        RCLCPP_ERROR(node_->get_logger(), "[WaitGripperStatus] 致命错误: 缺少 expected_status 参数");
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[WaitGripperStatus] 期待底层状态: %.1f", expected_status);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitGripperStatus::onRunning() {
    double expected_status, timeout_sec;
    if (!getInput("expected_status", expected_status) || !getInput("timeout", timeout_sec)) {
        return BT::NodeStatus::FAILURE; // 安全拦截
    }
    getInput("timeout", timeout_sec);

    // 检查是否达到期望状态 (0.0 或 2.0)
    if (std::abs(latest_gripper_status_ - expected_status) < 0.1) {
        RCLCPP_INFO(node_->get_logger(), "[WaitGripperStatus] 夹爪动作成功！当前状态: %.1f", latest_gripper_status_);
        return BT::NodeStatus::SUCCESS;
    }

    // STM32 返回 -1.0 表示物理异常（卡死/抓空）
    if (std::abs(latest_gripper_status_ - (-1.0)) < 0.1) {
        RCLCPP_ERROR(node_->get_logger(), "[WaitGripperStatus] 致命异常！STM32 汇报动作失败 (-1.0)！");
        return BT::NodeStatus::FAILURE;
    }

    // 检查超时
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_);
    if (elapsed.count() >= timeout_sec) {
        RCLCPP_ERROR(node_->get_logger(), "[WaitGripperStatus] 动作超时！(%.1fs) 当前状态停留在: %.1f", timeout_sec, latest_gripper_status_);
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitGripperStatus::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "[WaitGripperStatus] 节点被强行打断!");
}
#include "my_arm_automation/arm_trajectory_and_wait_node.hpp"

ArmTrajectoryAndWait::ArmTrajectoryAndWait(const std::string& name, const BT::NodeConfig& config, 
                                           std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group)
    : BT::StatefulActionNode(name, config), arm_group_(arm_group) 
{
}

BT::PortsList ArmTrajectoryAndWait::providedPorts() {
    // 改为接收字符串类型的 named pose (命名姿态)
    return { BT::InputPort<std::string>("target_pose_name") };
}

BT::NodeStatus ArmTrajectoryAndWait::onStart() {
    std::string target_name;
    
    if (!getInput("target_pose_name", target_name)) {
        RCLCPP_ERROR(rclcpp::get_logger("BT_ArmTrajectory"), "未获取到目标姿态名称(target_pose_name)!");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("BT_ArmTrajectory"), "准备移动到预设姿态: [%s]", target_name.c_str());

    // 核心修改点：直接把姿态名字喂给 MoveIt
    arm_group_->setNamedTarget(target_name);

    // 依然使用异步线程，防止 MoveIt 规划时卡死整个行为树心跳
    move_future_ = std::async(std::launch::async, [this]() {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            return (arm_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    });

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmTrajectoryAndWait::onRunning() {
    if (move_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        bool success = move_future_.get();
        if (success) {
            RCLCPP_INFO(rclcpp::get_logger("BT_ArmTrajectory"), "机械臂到达预设姿态点！");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("BT_ArmTrajectory"), "机械臂规划或运动失败！");
            return BT::NodeStatus::FAILURE; 
        }
    }
    return BT::NodeStatus::RUNNING;
}

void ArmTrajectoryAndWait::onHalted() {
    RCLCPP_WARN(rclcpp::get_logger("BT_ArmTrajectory"), "接收到打断信号，机械臂急停！");
    arm_group_->stop();
}
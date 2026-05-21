// 1. 引入对应的头文件
#include "my_arm_automation/move_arm_node.hpp"

// 引入逻辑需要的标准库 (注意：这些不需要放在头文件里，保持头文件清爽)
#include <sstream>
#include <vector>

// ---------------------------------------------------------
// 实现 1：构造函数
// 冒号 : 后面的叫“初始化列表”，在积木诞生的一瞬间，把 arm_group 存进私有变量 arm_group_ 中
// ---------------------------------------------------------
MoveArm::MoveArm(const std::string& name, const BT::NodeConfig& config, 
                 std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group)
    : BT::StatefulActionNode(name, config), arm_group_(arm_group) 
{
    // 括号里为空，因为初始化列表已经把活干完了
}

// ---------------------------------------------------------
// 实现 2：定义端口
// ---------------------------------------------------------
BT::PortsList MoveArm::providedPorts() {
    return { BT::InputPort<std::string>("target_joint_values") };
}

// ---------------------------------------------------------
// 实现 3：onStart() 逻辑 (当行为树第一次运行到这个积木时执行)
// ---------------------------------------------------------
BT::NodeStatus MoveArm::onStart() {
    std::string target_str;
    
    // 步骤 A：从黑板/XML中读取输入的字符串
    if (!getInput("target_joint_values", target_str)) {
        RCLCPP_ERROR(rclcpp::get_logger("BT_MoveArm"), "未获取到目标角度!");
        return BT::NodeStatus::FAILURE;
    }

    // 步骤 B：字符串切割 (把 "0.0; 1.5; -1.5; 0.0" 变成浮点数数组)
    std::vector<double> joint_group_positions;
    std::stringstream ss(target_str);
    std::string item;
    while (std::getline(ss, item, ';')) {
        try {
            joint_group_positions.push_back(std::stod(item));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("BT_MoveArm"), "角度字符串格式错误: %s", item.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    // 步骤 C：将目标角度喂给 MoveIt
    arm_group_->setJointValueTarget(joint_group_positions);
    RCLCPP_INFO(rclcpp::get_logger("BT_MoveArm"), "接收到新坐标，开始后台规划与运动...");

    // 步骤 D：开启异步线程 (这样 MoveIt 算路径时，行为树就不会卡死在这里)
    move_future_ = std::async(std::launch::async, [this]() {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            // 如果规划成功，则执行运动
            return (arm_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    });

    // 告诉行为树：“我已经在后台开干了，你下一个周期再来问我进度吧”
    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------
// 实现 4：onRunning() 逻辑 (处于 RUNNING 状态时，行为树每次 Tick 都会调用这里)
// ---------------------------------------------------------
BT::NodeStatus MoveArm::onRunning() {
    // 检查后台线程是否已经有了结果 (延时设为 0，意味着只看一眼，不等，立刻返回)
    if (move_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        
        // 获取后台线程的返回值 (true 成功，false 失败)
        bool success = move_future_.get();
        if (success) {
            RCLCPP_INFO(rclcpp::get_logger("BT_MoveArm"), "机械臂到达目标点！");
            return BT::NodeStatus::SUCCESS; // 告诉行为树这步完成了，可以走下一步了
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("BT_MoveArm"), "机械臂规划或运动失败！");
            return BT::NodeStatus::FAILURE; // 告诉行为树执行出错了，触发备用方案
        }
    }
    
    // 如果后台还没算完，继续保持 RUNNING 状态
    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------
// 实现 5：onHalted() 逻辑 (当行为树强行打断这个积木时触发)
// ---------------------------------------------------------
void MoveArm::onHalted() {
    RCLCPP_WARN(rclcpp::get_logger("BT_MoveArm"), "接收到打断信号，机械臂急停！");
    arm_group_->stop(); // 强制停止 MoveIt 运动
}
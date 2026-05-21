// 1. 头文件保护符 (Include Guards)
// 作用：防止这个文件在编译时被重复包含多次导致“重复定义”报错
#ifndef MY_ARM_AUTOMATION_MOVE_ARM_NODE_HPP
#define MY_ARM_AUTOMATION_MOVE_ARM_NODE_HPP

// 2. 引入必需的依赖
#include <behaviortree_cpp/action_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <future>

// 3. 类的声明 (继承自 StatefulActionNode)
class MoveArm : public BT::StatefulActionNode {
public:
    // 构造函数：创建这块积木时必须传入名字、配置、以及 MoveIt 控制句柄
    MoveArm(const std::string& name, const BT::NodeConfig& config, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group);

    // 静态函数：告诉行为树，这个积木需要外部输入一个字符串 (目标角度)
    static BT::PortsList providedPorts();

    // 重写行为树状态机函数 (这里只有声明，没有大括号 {} 里的逻辑)
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // 私有工具箱：只有这个积木内部能调用的变量
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_; // MoveIt 控制句柄
    std::future<bool> move_future_; // 用于后台异步监控机械臂运动状态
};

#endif // MY_ARM_AUTOMATION_MOVE_ARM_NODE_HPP
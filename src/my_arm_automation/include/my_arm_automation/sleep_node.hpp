#ifndef MY_ARM_AUTOMATION_SLEEP_NODE_HPP  // ← 头文件保护符，和 C 一样
#define MY_ARM_AUTOMATION_SLEEP_NODE_HPP

#include <behaviortree_cpp/action_node.h>  // ← 引入行为树框架
#include <rclcpp/rclcpp.hpp>  

class MySleep : public BT::SyncActionNode {  // ← 继承同步节点
public:
    // ① 构造函数：创建这个节点时自动调用
    MySleep(const std::string& name, 
            const BT::NodeConfig& config);

    // ② 告诉框架：我需要一个叫 duration 的输入参数
    static BT::PortsList providedPorts();

    // ③ tick 函数：被触发时执行的逻辑
    BT::NodeStatus tick() override;
};

#endif


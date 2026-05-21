#ifndef MY_ARM_AUTOMATION_FORCE_SUCCESS_NODE_HPP
#define MY_ARM_AUTOMATION_FORCE_SUCCESS_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

class ForceSuccess : public BT::SyncActionNode {
public:
    ForceSuccess(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    
    // 同步节点只需要重写 tick 函数
    BT::NodeStatus tick() override;
};

#endif // MY_ARM_AUTOMATION_FORCE_SUCCESS_NODE_HPP
#ifndef MY_ARM_AUTOMATION_FORCE_FAILURE_NODE_HPP
#define MY_ARM_AUTOMATION_FORCE_FAILURE_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

class ForceFailure : public BT::SyncActionNode {
public:
    ForceFailure(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

#endif // MY_ARM_AUTOMATION_FORCE_FAILURE_NODE_HPP
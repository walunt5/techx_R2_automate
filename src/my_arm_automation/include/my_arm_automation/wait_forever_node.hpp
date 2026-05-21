#ifndef MY_ARM_AUTOMATION_WAIT_FOREVER_NODE_HPP
#define MY_ARM_AUTOMATION_WAIT_FOREVER_NODE_HPP

#include <behaviortree_cpp/action_node.h>

class WaitForever : public BT::StatefulActionNode {
public:
    // 不需要传入 rclcpp::Node，因为它是纯逻辑节点
    WaitForever(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

#endif // MY_ARM_AUTOMATION_WAIT_FOREVER_NODE_HPP
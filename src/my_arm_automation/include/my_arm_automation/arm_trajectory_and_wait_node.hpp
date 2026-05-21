#ifndef MY_ARM_AUTOMATION_ARM_TRAJECTORY_AND_WAIT_NODE_HPP
#define MY_ARM_AUTOMATION_ARM_TRAJECTORY_AND_WAIT_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <future>

class ArmTrajectoryAndWait : public BT::StatefulActionNode {
public:
    ArmTrajectoryAndWait(const std::string& name, const BT::NodeConfig& config, 
                         std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::future<bool> move_future_;
};

#endif // MY_ARM_AUTOMATION_ARM_TRAJECTORY_AND_WAIT_NODE_HPP
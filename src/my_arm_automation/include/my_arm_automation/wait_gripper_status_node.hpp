#ifndef MY_ARM_AUTOMATION_WAIT_GRIPPER_STATUS_NODE_HPP
#define MY_ARM_AUTOMATION_WAIT_GRIPPER_STATUS_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp> // 替换为标准 Float64
#include <chrono>
#include <string>
#include <memory>

class WaitGripperStatus : public BT::StatefulActionNode {
public:
    WaitGripperStatus(const std::string& name, const BT::NodeConfig& config, 
                      std::shared_ptr<rclcpp::Node> node, 
                      const std::string& topic_name);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    // 订阅专属的状态码话题
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr status_sub_;
    
    double latest_gripper_status_ = 1.0; 
    std::chrono::steady_clock::time_point start_time_;

    // 回调函数变更
    void statusCallback(const std_msgs::msg::Float64::SharedPtr msg);
};

#endif // MY_ARM_AUTOMATION_WAIT_GRIPPER_STATUS_NODE_HPP
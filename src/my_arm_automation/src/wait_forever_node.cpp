#include "my_arm_automation/wait_forever_node.hpp"
#include <iostream>

WaitForever::WaitForever(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) 
{
}

BT::PortsList WaitForever::providedPorts() {
    return {}; // 不需要任何 XML 输入参数
}

BT::NodeStatus WaitForever::onStart() {
    std::cout << "[WaitForever] 节点启动，进入永久挂起状态..." << std::endl;
    return BT::NodeStatus::RUNNING; // 第一次执行就返回 RUNNING，卡住行为树
}

BT::NodeStatus WaitForever::onRunning() {
    // 行为树每次 Tick 到这里，依然返回 RUNNING
    return BT::NodeStatus::RUNNING; 
}

void WaitForever::onHalted() {
    std::cout << "[WaitForever] 收到打断信号，结束挂起！" << std::endl;
}
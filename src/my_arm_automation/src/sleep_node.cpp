#include "my_arm_automation/sleep_node.hpp"
#include <chrono>          // 时间相关的库
#include <thread>          // sleep_for 所在的库

// ① 构造函数：直接调用父类的构造函数，什么都不用做
MySleep::MySleep(const std::string& name, 
             const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {
}

// ② 端口定义：告诉框架"我需要一个叫 duration 的输入参数，类型是 double"
BT::PortsList MySleep::providedPorts() {
    return { BT::InputPort<double>("duration") };
}

// ③ tick 函数：核心逻辑
BT::NodeStatus MySleep::tick() {
    double duration;  // ← 声明一个 double 类型的变量，用来存"等多久"
    
    // 从行为树的输入端口读取 duration 参数
    // 如果读不到（比如 XML 里没写 duration），就报错返回 FAILURE
    if (!getInput("duration", duration)) {
        RCLCPP_ERROR(rclcpp::get_logger("BT_Sleep"), 
                     "没有获取到 duration 参数！");
        return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("BT_Sleep"), 
                "休眠 %.1f 秒...", duration);
    
    // 核心：让程序停在这里等 duration 秒
    // std::chrono::milliseconds(x) 意思是"x 毫秒"
    // duration * 1000 是把秒转成毫秒
    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            static_cast<int>(duration * 1000)));
    
    // 睡完了，返回成功
    return BT::NodeStatus::SUCCESS;
}

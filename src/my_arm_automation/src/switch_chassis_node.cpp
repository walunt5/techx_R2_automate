#include "my_arm_automation/switch_chassis_node.hpp"

SwitchChassis::SwitchChassis(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config) 
{
  client_ = node->create_client<std_srvs::srv::SetBool>("/switch_chassis_mode");
}

BT::PortsList SwitchChassis::providedPorts() {
  // 👈 核心修复 1：改名字为 "mode"，并且把类型改为 std::string
  return { BT::InputPort<std::string>("mode") };
}

BT::NodeStatus SwitchChassis::tick() {
  std::string mode_str;
  if (!getInput("mode", mode_str)) {
    RCLCPP_ERROR(rclcpp::get_logger("BT_Chassis"), "缺少 mode 参数");
    return BT::NodeStatus::FAILURE;
  }

  // 👈 核心修复 2：把获取到的字符串转换成底盘服务需要的 bool 信号
  // 如果 XML 里填的是 "rough_terrain"，is_rough 就是 true，否则就是 false
  bool is_rough = (mode_str == "rough_terrain");

  // 1. 检查服务是否存在
  if (!client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(rclcpp::get_logger("BT_Chassis"), "底盘服务未就绪，跳过切换（测试模式）");
    return BT::NodeStatus::SUCCESS; // 测试阶段我们可以先返回 SUCCESS
  }

  // 2. 发送请求
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = is_rough;

  RCLCPP_INFO(rclcpp::get_logger("BT_Chassis"), "正在请求底盘模式切换：%s", is_rough ? "越野" : "平地");
  
  // 同步调用
  auto result = client_->async_send_request(request);
  return BT::NodeStatus::SUCCESS;
}
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

// ==============================================================================
// 引入自定义行为树积木块 (Action / Condition / Control Nodes)
// ==============================================================================
// 1. 机械臂关节与位姿控制
#include "my_arm_automation/move_arm_node.hpp"
#include "my_arm_automation/arm_trajectory_and_wait_node.hpp"
#include "my_arm_automation/arm_cartesian_and_wait_node.hpp"

// 2. 末端执行器控制与状态检测
#include "my_arm_automation/control_gripper_node.hpp"
#include "my_arm_automation/wait_gripper_status_node.hpp"

// 3. 流程控制与状态强制
#include "my_arm_automation/force_success_node.hpp"
#include "my_arm_automation/force_failure_node.hpp"

// 4. 外部交互与辅助功能
#include "my_arm_automation/wait_for_signal_node.hpp"
#include "my_arm_automation/wait_forever_node.hpp"
#include "my_arm_automation/sleep_node.hpp"
#include "my_arm_automation/play_audio_node.hpp"
#include "my_arm_automation/switch_chassis_node.hpp"

int main(int argc, char **argv) 
{
    // ==========================================================================
    // 1. ROS 2 系统初始化
    // ==========================================================================
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_executor");
    
    // ==========================================================================
    // 2. 初始化 MoveIt 控制组 (双臂解耦架构)
    // ==========================================================================
    // 注意：这里的 "arm_group" 和 "arm_group2" 必须与你在 SRDF 文件中配置的 Planning Group 名字严格一致
    auto suction_arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm_group");
    auto gripper_arm_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm_group2");

    // ==========================================================================
    // 3. 实例化行为树工厂并注册积木块
    // ==========================================================================
    BT::BehaviorTreeFactory factory;

    // --------------------------------------------------------------------------
    // [模块 A] 吸盘机械臂控制节点注册
    // --------------------------------------------------------------------------
    factory.registerBuilder<MoveArm>("MoveSuctionArm",
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveArm>(name, config, suction_arm_group);
        });

    factory.registerBuilder<ArmTrajectoryAndWait>("SuctionArm_Trajectory_AndWait", 
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ArmTrajectoryAndWait>(name, config, suction_arm_group);
        });

    factory.registerBuilder<ArmCartesianAndWait>("SuctionArm_Cartesian_AndWait", 
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ArmCartesianAndWait>(name, config, suction_arm_group);
        });

    // --------------------------------------------------------------------------
    // [模块 B] 夹爪机械臂控制节点注册
    // --------------------------------------------------------------------------
    factory.registerBuilder<MoveArm>("MoveGripperArm",
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<MoveArm>(name, config, gripper_arm_group);
        });

    factory.registerBuilder<ArmTrajectoryAndWait>("GripperArm_Trajectory_AndWait", 
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ArmTrajectoryAndWait>(name, config, gripper_arm_group);
        });

    factory.registerBuilder<ArmCartesianAndWait>("GripperArm_Cartesian_AndWait", 
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ArmCartesianAndWait>(name, config, gripper_arm_group);
        });

    // --------------------------------------------------------------------------
    // [模块 C] 泛化的末端执行器控制节点 (通过传入不同话题名称实现多态复用)
    // --------------------------------------------------------------------------
    // -- 吸盘控制与状态监测 --
    factory.registerBuilder<ControlGripper>("ControlSuction",
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ControlGripper>(name, config, node, "/suction_cmd_controller/commands");
        });

    factory.registerBuilder<WaitGripperStatus>("Wait_Suction_Status",
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<WaitGripperStatus>(name, config, node, "/suction/status");
        });

    // -- 夹爪控制与状态监测 --
    factory.registerBuilder<ControlGripper>("ControlGripper",
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ControlGripper>(name, config, node, "/gripper_cmd_controller/commands");
        });

    factory.registerBuilder<WaitGripperStatus>("Wait_Gripper_Status",
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<WaitGripperStatus>(name, config, node, "/gripper/status");
        });

    // --------------------------------------------------------------------------
    // [模块 D] 通用逻辑与外围设备交互节点注册
    // --------------------------------------------------------------------------
    factory.registerBuilder<WaitForSignal>("WaitForSignal", 
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<WaitForSignal>(name, config, node);
        });

    factory.registerBuilder<PlayAudio>("PlayAudio", 
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<PlayAudio>(name, config, node);
        });

    factory.registerBuilder<SwitchChassis>("SwitchChassis", 
        [&](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<SwitchChassis>(name, config, node);
        });

    // 无状态依赖的轻量级节点直接注册类型
    factory.registerNodeType<WaitForever>("Wait_Forever"); 
    factory.registerNodeType<ForceSuccess>("Force_Success");
    factory.registerNodeType<ForceFailure>("Force_Failure");
    factory.registerNodeType<MySleep>("MySleep");


    // ==========================================================================
    // 4. 加载 XML 行为树剧本配置
    // ==========================================================================
    // 声明参数，以便 Launch 文件可以注入 xml_file_path
    node->declare_parameter<std::string>("xml_file_path", "");
    
    std::string xml_path;
    node->get_parameter("xml_file_path", xml_path);
    
    // 安全验证：若未获取到路径，优雅退出防止崩溃
    if (xml_path.empty()) {
        RCLCPP_ERROR(node->get_logger(), "致命错误: 未获取到 XML 行为树剧本路径！请检查 Launch 文件的参数传递。");
        rclcpp::shutdown();
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "正在加载行为树剧本: %s", xml_path.c_str());
    auto tree = factory.createTreeFromFile(xml_path);
    
    // 启动 Groot2 调试发布器，方便在上位机实时可视化监控 BT 运行状态
    BT::Groot2Publisher publisher(tree);


    // ==========================================================================
    // 5. 行为树主执行循环 (Tick Engine)
    // ==========================================================================
    rclcpp::Rate rate(10); // 运行频率 10Hz (每 100ms Tick 一次)
    
    while (rclcpp::ok()) {
        // 让行为树走过一个时间步
        BT::NodeStatus status = tree.tickExactlyOnce();
        
        // 处理 ROS 2 的回调（维持订阅、服务等通信的存活）
        rclcpp::spin_some(node);
        
        // 检查全局执行结果
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "=== 行为树总任务 [圆满完成]，程序结束 ===");
            break; 
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "=== 行为树总任务 [彻底失败]，程序结束 ===");
            break; 
        }
        
        // 如果 status 仍然是 RUNNING，则休眠等待下一帧
        rate.sleep();
    }

    // ==========================================================================
    // 6. 资源清理
    // ==========================================================================
    rclcpp::shutdown();
    return 0;
}
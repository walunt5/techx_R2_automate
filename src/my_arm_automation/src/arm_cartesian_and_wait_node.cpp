#include "my_arm_automation/arm_cartesian_and_wait_node.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

ArmCartesianAndWait::ArmCartesianAndWait(const std::string& name, const BT::NodeConfig& config, 
                                         std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group)
    : BT::StatefulActionNode(name, config), arm_group_(arm_group) 
{
}

BT::PortsList ArmCartesianAndWait::providedPorts() {
    return { 
        BT::InputPort<std::string>("axis"),      // 移动轴: X, Y, Z
        BT::InputPort<double>("dist"),           // 距离 (米)
        BT::InputPort<std::string>("frame_type") // 坐标系类型: "base" (世界) 或 "local" (末端)
    };
}

BT::NodeStatus ArmCartesianAndWait::onStart() {
    std::string axis, frame_type;
    double dist = 0.0;
    
    if (!getInput("axis", axis) || !getInput("dist", dist) || !getInput("frame_type", frame_type)) {
        RCLCPP_ERROR(rclcpp::get_logger("BT_Cartesian"), "参数缺失！需 axis, dist, frame_type");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("BT_Cartesian"), "开始直线规划: 轴[%s], 距离[%.3f], 坐标系[%s]", 
                axis.c_str(), dist, frame_type.c_str());

    move_future_ = std::async(std::launch::async, [this, axis, dist, frame_type]() {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        
        // 1. 精准计算终点位姿 (依靠 TF2)
        if (frame_type == "local") {
            tf2::Transform transform;
            tf2::fromMsg(start_pose, transform);
            tf2::Vector3 local_vec(0.0, 0.0, 0.0);
            
            if (axis == "X" || axis == "x") local_vec.setX(dist);
            else if (axis == "Y" || axis == "y") local_vec.setY(dist);
            else if (axis == "Z" || axis == "z") local_vec.setZ(dist);
            
            tf2::Vector3 global_vec = transform.getBasis() * local_vec;
            target_pose.position.x += global_vec.x();
            target_pose.position.y += global_vec.y();
            target_pose.position.z += global_vec.z();
        } else {
            if (axis == "X" || axis == "x") target_pose.position.x += dist;
            else if (axis == "Y" || axis == "y") target_pose.position.y += dist;
            else if (axis == "Z" || axis == "z") target_pose.position.z += dist;
        }

        // ==== 核心修复 1：不要手动插值！只给 MoveIt 终点 ====
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose); // 让 MoveIt 自己去切分步长！

        // 降低执行速度防止震动
        arm_group_->setMaxVelocityScalingFactor(0.8);
        arm_group_->setMaxAccelerationScalingFactor(0.5);
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.03;
        
        // ==== 核心修复 2：必须禁用跳跃阈值 ====
        const double jump_threshold = 0.0; // 0.0 表示禁用！这是防止误报的最关键参数！

        double fraction = arm_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction >= 0.85) {
            RCLCPP_INFO(rclcpp::get_logger("BT_Cartesian"), "直线规划成功: %.1f%%，开始进行时间参数化平滑处理", fraction * 100.0);
    
            // ==== 修复：加入数据转换桥梁 ====
            // 1. 创建一个 MoveIt 内部的“智能轨迹类”，绑定你的机器人模型和规划组
            robot_trajectory::RobotTrajectory rt(arm_group_->getRobotModel(), arm_group_->getName());
            
            // 2. 以机器人当前状态为参考，把刚才算出来的无脑 msg 塞进这个类里
            rt.setRobotTrajectoryMsg(*arm_group_->getCurrentState(), trajectory);

            // 3. 引入 TOTG (时间最优轨迹生成) 算法重构时间轴
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            
            // 4. 计算时间戳！参数: (轨迹对象, 全局最大速度比例, 全局最大加速度比例)
            bool success_totg = totg.computeTimeStamps(rt, 0.5, 0.5);

            // 5. 算法处理完后，再把它转回普通的 ROS msg 格式，准备下发给控制器
            rt.getRobotTrajectoryMsg(trajectory);
            // ==== 转换结束 ====

            if (success_totg) {
                RCLCPP_INFO(rclcpp::get_logger("BT_Cartesian"), "轨迹时间参数化成功！下发执行...");
                return (arm_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("BT_Cartesian"), "轨迹时间参数化失败！");
                return false;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("BT_Cartesian"), "直线规划失败(仅%.1f%%)，启用容错姿态规划", fraction * 100.0);
            
            // ==== 核心修复 3：放宽备用方案的姿态约束 ====
            // 如果是少于 6 DOF 的机械臂，放宽目标点的姿态容差（允许手腕稍微歪一点点，换取能够到达目标位置）
            arm_group_->setGoalOrientationTolerance(0.1); // 允许约 5 度的姿态误差
            arm_group_->setGoalPositionTolerance(0.01);   // 允许 1cm 的位置误差
            
            arm_group_->setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
            
            bool success = (arm_group_->plan(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            // 恢复默认容差（避免影响后续动作）
            arm_group_->setGoalOrientationTolerance(0.001);
            arm_group_->setGoalPositionTolerance(0.0001);
            
            if (success) {
                RCLCPP_INFO(rclcpp::get_logger("BT_Cartesian"), "容错关节规划成功！开始执行兜底路径。");
                return (arm_group_->execute(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("BT_Cartesian"), "致命错误：目标点在物理上完全无法到达！请检查预设点是否太靠近极限。");
                return false;
            }
        }
    });
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmCartesianAndWait::onRunning() {
    if (move_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        return move_future_.get() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void ArmCartesianAndWait::onHalted() {
    arm_group_->stop();
}
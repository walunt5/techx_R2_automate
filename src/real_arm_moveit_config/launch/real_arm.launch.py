import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # =====================================================================
    # 1. 核心配置：构建 MoveIt 环境 (明确指向真实物理硬件)
    # =====================================================================
    moveit_config = (
        MoveItConfigsBuilder("real_arm", package_name="real_arm_moveit_config")
        .robot_description(mappings={"use_sim": "false"}) 
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 获取 RViz 配置文件的绝对路径
    rviz_config_file = os.path.join(
        get_package_share_directory("real_arm_moveit_config"),
        "config",
        "moveit.rviz",
    )

    # =====================================================================
    # 2. 定义所有基础节点
    # =====================================================================
    
    # A. 启动 robot_state_publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # B. 启动底层的 ros2_control 节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("real_arm_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="screen",
    )

    # C. 挂载关节状态广播器 (JSB)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # =====================================================================
    # 3. 挂载实际工程中的 4 个核心控制器
    # =====================================================================
    # D1. 吸盘臂轨迹控制器 (对应 arm_group)
    arm_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # D2. 夹爪臂轨迹控制器 (对应 arm_group2)
    arm_group2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group2_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    # D3. 吸盘气泵命令控制器 (ForwardCommandController)
    suction_cmd_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["suction_cmd_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # D4. 电动夹爪命令控制器 (ForwardCommandController)
    gripper_cmd_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_cmd_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # =====================================================================
    # 4. MoveIt 与 RViz 节点
    # =====================================================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # =====================================================================
    # 5. 构建高可用启动时序 (Event Handlers)
    # =====================================================================
    # 确保 JSB 启动完毕，获取到硬件真实位置后，再启动这 4 个控制器
    delay_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg="关节状态广播器(JSB)启动完毕，开始挂载双臂与末端控制器..."),
                arm_group_controller_spawner,
                arm_group2_controller_spawner,
                suction_cmd_controller_spawner,
                gripper_cmd_controller_spawner,
            ],
        )
    )

    # 等待吸盘臂控制器加载完毕后，再启动 MoveGroup 和 RViz
    delay_movegroup_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_group_controller_spawner,
            on_exit=[
                LogInfo(msg="底层控制器挂载完毕，启动 MoveIt2 规划核心与 RViz..."),
                move_group_node,
                rviz_node,
            ],
        )
    )

    # =====================================================================
    # 6. 返回 Launch 描述符
    # =====================================================================
    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_controllers_after_jsb,
        delay_movegroup_after_controllers,
    ])
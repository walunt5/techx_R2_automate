import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载配置并修复底层 Tuple Bug
    moveit_config = (
        MoveItConfigsBuilder("real_arm", package_name="real_arm_moveit_config")
        .robot_description(mappings={"use_sim": "true"})
        .to_moveit_configs()
    )
    moveit_config_dict = moveit_config.to_dict()
    def clean_dict(target_dict):
        for k, v in list(target_dict.items()):
            if isinstance(v, tuple):
                target_dict[k] = "" if len(v) == 0 else list(v)
            elif isinstance(v, dict):
                clean_dict(v)
        return target_dict
    moveit_config_dict = clean_dict(moveit_config_dict)

    # 2. 启动 Gazebo 物理引擎
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 3. 状态发布节点
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # 4. 在 Gazebo 中生成实体机械臂
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'real_arm', '-z', '0.0'],
        output='screen'
    )

    # 5. MoveGroup 节点 (大脑)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict, {"use_sim_time": True}],
    )

    # 6. RViz2 可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("real_arm_moveit_config"), "config", "moveit.rviz")],
        parameters=[moveit_config_dict, {"use_sim_time": True}],
    )

    # 7. 底层电机控制器 (已根据 YAML 配置文件修正)
    spawn_broadcaster = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )
    spawn_arm_controller = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["arm_group_controller", "--controller-manager", "/controller_manager"]
    )
    spawn_arm2_controller = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["arm_group2_controller", "--controller-manager", "/controller_manager"]
    )

    # 8. 严格时序：模型生成后 -> 启动广播 -> 启动两个控制器
    delay_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[spawn_broadcaster])
    )
    delay_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_broadcaster, 
            on_exit=[spawn_arm_controller, spawn_arm2_controller]
        )
    )

    return LaunchDescription([
        gazebo, rsp_node, spawn_entity, move_group_node, rviz_node,
        delay_broadcaster, delay_controllers
    ])
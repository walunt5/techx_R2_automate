import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 获取行为树 XML 文件的路径
    package_path = get_package_share_directory('my_arm_automation')
    # 假设你的 XML 放在 config 目录下
    default_xml_path = os.path.join(package_path, 'config', 'test_gym_master_updated.xml')

    # 2. 获取 MoveIt 配置（机械臂控制必需的 robot_description 等参数）
    # 这里需要指向你之前生成的 moveit_config 功能包
    moveit_config = MoveItConfigsBuilder("real_arm", package_name="real_arm_moveit_config").to_moveit_configs()

    # 3. 定义行为树主执行节点
    bt_node = Node(
        package='my_arm_automation',
        executable='bt_automation_node',  # 必须与 CMakeLists.txt 中的 add_executable 一致
        name='bt_executor_node',
        output='screen',
        # 将 MoveIt 的模型参数和行为树路径参数合并传递
        parameters=[
            moveit_config.to_dict(),
            {'xml_file_path': default_xml_path},
            {'use_sim_time': True}  # 如果在仿真环境运行，请设为 True
        ]
    )

    return LaunchDescription([
        bt_node
    ])
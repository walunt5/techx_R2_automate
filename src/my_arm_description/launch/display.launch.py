import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('my_arm_description')

    # 默认 xacro 文件路径
    default_model_path = os.path.join(pkg_share, 'urdf', 'real_arm.urdf.xacro')

    # 默认 RViz 配置文件路径
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'display1.rviz')

    # 声明启动参数，允许运行时通过 model:=xxx 指定其他 xacro 文件
    declare_model_path = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to the robot urdf.xacro file'
    )

    # 声明 RViz 配置参数
    declare_rviz_config = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )

    # Robot State Publisher 节点
    # 它会将 urdf 模型加载到参数服务器，并通过 /tf 和 /tf_static 发布坐标系变换
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str
            )
        }]
    )

    # Joint State Publisher GUI 节点
    # 提供一个带有滑块的 GUI 窗口，方便手动调节各个关节角度
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 节点
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        declare_model_path,
        declare_rviz_config,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ])

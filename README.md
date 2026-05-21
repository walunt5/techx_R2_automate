# R2 双机械臂 ROS 2 工作空间

这个工程是一个基于 ROS 2、MoveIt 2、ros2_control 和 BehaviorTree.CPP 的双机械臂控制工作空间。它包含机械臂模型显示、真实硬件串口控制、MoveIt 规划执行、夹爪/吸盘末端控制，以及面向任务流程的行为树自动化节点。

当前工作空间中还有 STM32 固件与通信协议代码，用于配合上位机 ROS 2 控制节点完成机械臂底层状态回传和指令下发。

## 工程结构

```text
.
├── src/
│   ├── my_arm_description/        # 机械臂 URDF/Xacro、网格模型、RViz 显示 launch
│   ├── my_arm_hardware/           # ros2_control 真实硬件接口插件，串口连接 STM32
│   ├── real_arm_moveit_config/    # MoveIt 2 配置、SRDF、控制器配置、真实机械臂启动文件
│   └── my_arm_automation/         # 行为树自动化执行节点和任务 XML
├── protocol7/                     # 上位机/下位机通信协议 C 代码
├── arm1.0 (3)/                    # STM32 工程源码
└── R2_Robot_UART_Circular_DMA_Protocol_Guide.pdf
```

## ROS 2 功能包说明

| 功能包 | 作用 |
| --- | --- |
| `my_arm_description` | 提供 `real_arm.urdf.xacro`、STL 模型和 `display.launch.py`，用于单独显示机器人模型。 |
| `my_arm_hardware` | 提供 `my_arm_hardware/ArmHardwareInterface` 硬件插件，通过串口和 STM32 通信。 |
| `real_arm_moveit_config` | MoveIt 2 配置包，包含规划组、控制器、RViz 和真实硬件启动文件。 |
| `my_arm_automation` | 行为树执行包，封装机械臂运动、夹爪/吸盘控制、信号等待、底盘切换、音频播放等任务节点。 |

## 机械臂与控制接口

MoveIt 规划组定义在 `src/real_arm_moveit_config/config/real_arm.srdf`：

| 规划组 | 关节 | 说明 |
| --- | --- | --- |
| `arm_group` | `big_arm_revolute_joint`, `small_arm_revolute_joint` | 吸盘侧机械臂 |
| `arm_group2` | `big_arm_revolute_joint2` | 夹爪侧机械臂 |
| `hand_group` | `suction_cup_link` | 吸盘末端 |
| `gripper` | `gripper_link2` | 夹爪末端 |

ros2_control 硬件配置位于 `src/my_arm_description/urdf/real_arm.urdf.xacro`：

```xml
<plugin>my_arm_hardware/ArmHardwareInterface</plugin>
<param name="port">/dev/ttyUSB0</param>
<param name="baudrate">115200</param>
```

默认电机映射：

| motor_id | 关节 |
| --- | --- |
| `1` | `big_arm_revolute_joint` |
| `2` | `small_arm_revolute_joint` |
| `3` | `big_arm_revolute_joint2` |
| `4` | `suction_joint` |
| `5` | `gripper_joint` |

控制器配置位于 `src/real_arm_moveit_config/config/ros2_controllers.yaml`：

- `arm_group_controller`：吸盘侧机械臂轨迹控制器
- `arm_group2_controller`：夹爪侧机械臂轨迹控制器
- `suction_cmd_controller`：吸盘命令控制器
- `gripper_cmd_controller`：夹爪命令控制器
- `joint_state_broadcaster`：关节状态发布器

## 主要话题和服务

| 名称 | 类型/用途 |
| --- | --- |
| `/joint_states` | ros2_control 发布的关节状态 |
| `/gripper/status` | STM32 或模拟器发布的夹爪状态，`std_msgs/msg/Float64` |
| `/suction/status` | STM32 或模拟器发布的吸盘状态，`std_msgs/msg/Float64` |
| `/gripper_cmd_controller/commands` | 夹爪控制命令，`std_msgs/msg/Float64MultiArray` |
| `/suction_cmd_controller/commands` | 吸盘控制命令，`std_msgs/msg/Float64MultiArray` |
| `/vision/signals` | 行为树等待的外部导航/视觉/任务信号，`std_msgs/msg/String` |
| `/audio_play` | 行为树发布的音频播放请求，`std_msgs/msg/String` |
| `/switch_chassis_mode` | 行为树调用的底盘模式切换服务，`std_srvs/srv/SetBool` |

状态码约定：

| 状态值 | 含义 |
| --- | --- |
| `0.0` | 张开/释放到位 |
| `1.0` | 动作中或吸附成功，具体含义看夹爪/吸盘分支 |
| `2.0` | 夹爪抓紧完成 |
| `-1.0` | 物理异常，例如卡死、抓空或气压异常 |
| `9.9` | 未知或空闲初始状态 |

## 环境依赖

以 ROS 2 Humble 为例：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-behaviortree-cpp \
  ros-humble-tf2-geometry-msgs \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2
```

也可以优先用 rosdep 安装依赖：

```bash
cd /home/xie/xjb_demo_test/new_arm_kfs_ws_2
rosdep install --from-paths src --ignore-src -r -y
```

如果要连接真实 STM32，当前用户需要有串口权限：

```bash
sudo usermod -a -G dialout $USER
```

执行后需要重新登录终端或重启系统。如果实际串口不是 `/dev/ttyUSB0`，修改：

```text
src/my_arm_description/urdf/real_arm.urdf.xacro
```

## 编译

```bash
cd /home/xie/xjb_demo_test/new_arm_kfs_ws_2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

查看工作空间内的 ROS 2 包：

```bash
colcon list
```

本工程当前包含：

```text
my_arm_automation
my_arm_description
my_arm_hardware
real_arm_moveit_config
```

## 启动模型显示

只查看 URDF 模型和关节滑块，不连接真实硬件：

```bash
cd /home/xie/xjb_demo_test/new_arm_kfs_ws_2
source install/setup.bash
ros2 launch my_arm_description display.launch.py
```

这个 launch 会启动：

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

## 启动真实机械臂 MoveIt 控制

确认 STM32 已连接，串口参数正确后运行：

```bash
cd /home/xie/xjb_demo_test/new_arm_kfs_ws_2
source install/setup.bash
ros2 launch real_arm_moveit_config real_arm.launch.py
```

这个 launch 会按顺序启动：

1. `robot_state_publisher`
2. `ros2_control_node`
3. `joint_state_broadcaster`
4. `arm_group_controller`
5. `arm_group2_controller`
6. `suction_cmd_controller`
7. `gripper_cmd_controller`
8. `move_group`
9. `rviz2`

启动后可以检查控制器状态：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic echo /gripper/status
ros2 topic echo /suction/status
```

## 启动行为树自动化

行为树默认 XML 是：

```text
src/my_arm_automation/config/test_gym_master_updated.xml
```

先启动真实机械臂 MoveIt 控制：

```bash
ros2 launch real_arm_moveit_config real_arm.launch.py
```

再开一个终端运行行为树：

```bash
cd /home/xie/xjb_demo_test/new_arm_kfs_ws_2
source install/setup.bash
ros2 launch my_arm_automation bt_automation.launch.py
```

当前行为树包含的主要流程：

- 等待导航到位信号 `NAV_SUCCESS`
- 等待视觉对准信号 `VISION_SUCCESS`
- 控制夹爪机械臂到 `ready2`
- 下发夹爪闭合命令
- 等待 `/gripper/status` 返回期望状态
- 控制吸盘侧机械臂到 `ready` 或 `state1`
- 等待组装完成信号 `ASSEMBLY_DONE`
- 收到 `LEAVE_GYM` 后切换底盘模式并进入转场流程

## 使用模拟器测试行为树

工程里提供了一个简单的状态和信号模拟脚本：

```bash
cd /home/xie/xjb_demo_test/new_arm_kfs_ws_2
source install/setup.bash
python3 src/my_arm_automation/scripts/mock_simulator.py
```

模拟器会发布：

- `/vision/signals`
- `/gripper/status`
- `/suction/status`

键盘输入示例：

| 输入 | 发布内容 |
| --- | --- |
| `N` | `NAV_SUCCESS` |
| `V` | `VISION_SUCCESS` |
| `A` | `ASSEMBLY_DONE` |
| `L` | `LEAVE_GYM` |
| `2` | 夹爪状态 `2.0` |
| `0` | 夹爪状态 `0.0` |
| `S1` | 吸盘状态 `1.0` |
| `S0` | 吸盘状态 `0.0` |

## 常见修改位置

| 需求 | 文件 |
| --- | --- |
| 修改串口号或波特率 | `src/my_arm_description/urdf/real_arm.urdf.xacro` |
| 修改控制器参数 | `src/real_arm_moveit_config/config/ros2_controllers.yaml` |
| 修改 MoveIt 控制器映射 | `src/real_arm_moveit_config/config/moveit_controllers.yaml` |
| 修改规划组和命名姿态 | `src/real_arm_moveit_config/config/real_arm.srdf` |
| 修改默认行为树 XML | `src/my_arm_automation/launch/bt_automation.launch.py` |
| 修改行为树任务流程 | `src/my_arm_automation/config/test_gym_master_updated.xml` |
| 修改硬件串口协议解析 | `src/my_arm_hardware/src/arm_hardware_interface.cpp` |

## 注意事项

- `real_arm.launch.py` 当前面向真实硬件，`use_sim` 被设置为 `false`。
- 启动真实硬件前，建议先确认 STM32 正在持续回传有效关节数据。
- `my_arm_hardware` 在激活时会等待最多 3 秒同步 STM32 初始状态；如果没有收到有效帧，终端会提示存在启动风险。
- 行为树节点依赖 MoveIt 的 `arm_group` 和 `arm_group2`，因此运行行为树前需要先保证 `move_group` 和底层控制器已正常启动。
- `bt_automation.launch.py` 当前把 `use_sim_time` 设置为 `True`；如果在真实机器人上不使用仿真时间，可以按实际需求改为 `False`。

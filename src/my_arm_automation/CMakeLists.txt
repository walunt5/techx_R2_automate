cmake_minimum_required(VERSION 3.8)
project(my_arm_automation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 1. 告诉 CMake：我要把你列出来的这4个 cpp 文件，打包编译成 1 个叫 bt_automation_node 的程序
add_executable(bt_automation_node
  src/main_tree_executor.cpp
  src/move_arm_node.cpp
  src/control_gripper_node.cpp
  src/wait_for_signal_node.cpp
  src/wait_forever_node.cpp
  src/force_success_node.cpp
  src/force_failure_node.cpp
  src/play_audio_node.cpp
  src/switch_chassis_node.cpp
  src/arm_trajectory_and_wait_node.cpp
  src/arm_cartesian_and_wait_node.cpp
  src/sleep_node.cpp
  src/wait_gripper_status_node.cpp
)

# 2. 告诉 CMake：我的头文件 (.hpp) 放在了 include 文件夹里，你要去这里找
target_include_directories(bt_automation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. 把所有的依赖库链接给这个程序 (这里把你之前写的依赖加上就行)
ament_target_dependencies(bt_automation_node
  rclcpp
  moveit_ros_planning_interface
  behaviortree_cpp
  std_msgs
  std_srvs
  sensor_msgs
  tf2_ros           
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 4. 确保安装路径正确
install(TARGETS bt_automation_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装整个 launch config 文件夹
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

#ifndef MY_ARM_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_
#define MY_ARM_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// 1. 引入标准浮点消息类型
#include "std_msgs/msg/float64.hpp"

namespace my_arm_hardware
{
class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int serial_fd_ = -1;
  bool open_serial(const std::string & port, int baudrate);
  void close_serial();
  bool send_command(const std::vector<uint8_t> & data);
  std::vector<double> read_joint_angles_from_serial(bool& frame_parsed);
  
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_commands_;
  std::string port_;
  int baudrate_ = 115200;
  std::vector<int> motor_ids_;
  rclcpp::Logger logger_{rclcpp::get_logger("ArmHardwareInterface")};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
  std::vector<double> last_positions_;
  std::vector<double> filtered_positions_;
  std::vector<double> lowpass_alpha_;
  rclcpp::Time last_read_time_;
  rclcpp::Time last_valid_time_;
  std::deque<uint8_t> serial_buffer_;
  bool parse_serial_buffer(std::vector<double> & positions);

  // 2. 声明 ROS 2 节点和状态发布器
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr suction_status_pub_;
  double last_published_gripper_status_ = 9.9;
  double last_published_suction_status_ = 9.9;
  bool has_valid_data_ = false;
};

}  // namespace my_arm_hardware

#endif  // MY_ARM_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_
#include "my_arm_hardware/arm_hardware_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <algorithm> 
#include <cmath>
#include <chrono>

namespace my_arm_hardware
{
hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    port_ = info_.hardware_parameters.at("port");
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Missing hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  size_t joint_count = info_.joints.size();
  joint_positions_.resize(joint_count, 0.0);
  joint_velocities_.resize(joint_count, 0.0);
  joint_commands_.resize(joint_count, 0.0);
  motor_ids_.resize(joint_count, 1);
  last_positions_.resize(joint_count, 0.0);
  filtered_positions_.resize(joint_count, 0.0);
  lowpass_alpha_.resize(joint_count, 0.3);
  last_read_time_ = rclcpp::Time(0, 0);
  last_valid_time_ = rclcpp::Time(0, 0);

  for (size_t i = 0; i < joint_count; ++i) {
    const auto & joint = info_.joints[i];
    if (joint.parameters.find("motor_id") != joint.parameters.end()) {
      motor_ids_[i] = std::stoi(joint.parameters.at("motor_id"));
    } else {
      motor_ids_[i] = static_cast<int>(i + 1); 
    }

    if (joint.name.find("big_arm") != std::string::npos) {
      lowpass_alpha_[i] = 0.40;
    } else if (joint.name == "suction_joint" || joint.name == "gripper_joint") {
      lowpass_alpha_[i] = 0.5;
    } else {
      lowpass_alpha_[i] = 0.3;
    }
  }

  // --- 初始化 ROS 2 内部节点并创建发布器 ---
  // 由于硬件接口在自己的生命周期中管理，我们需要一个内部节点来发布常规话题
  node_ = std::make_shared<rclcpp::Node>("arm_hardware_internal_node");
  gripper_status_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/gripper/status", 10);
  suction_status_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/suction/status", 10);
  RCLCPP_INFO(logger_, "成功初始化夹爪状态发布器: /gripper/status");
  RCLCPP_INFO(logger_, "成功初始化吸盘状态发布器: /suction/status");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring... opening serial port");
  if (!open_serial(port_, baudrate_)) {
    RCLCPP_ERROR(logger_, "Failed to open serial port");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating...");

  auto start_wait = std::chrono::steady_clock::now();
  bool first_frame_received = false;
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_wait).count() < 3)
  {
      read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0));
      if (has_valid_data_) {
          first_frame_received = true;
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!first_frame_received) {
      RCLCPP_WARN(logger_, "启动时未能获取到 STM32 的初始状态，可能存在乱飞风险！");
  } else {
      RCLCPP_INFO(logger_, "已成功同步机械臂初始物理状态。");
  }

  // 同步指令和状态
  for (size_t i = 0; i < joint_positions_.size(); i++) {
    joint_commands_[i] = joint_positions_[i];
    filtered_positions_[i] = joint_positions_[i];
  }
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating... closing serial port");
  close_serial();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  bool frame_parsed = false;
  std::vector<double> actual_positions = read_joint_angles_from_serial(frame_parsed);
  has_valid_data_ = frame_parsed;

  // 识别夹爪和吸盘虚拟关节索引
  int gripper_idx = -1;
  int suction_idx = -1;

  if (frame_parsed) {
    last_valid_time_ = time;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (frame_parsed) {
      double a = lowpass_alpha_[i];
      filtered_positions_[i] = a * actual_positions[i] + (1.0 - a) * filtered_positions_[i];
    }
    joint_positions_[i] = filtered_positions_[i];

    if (info_.joints[i].name == "gripper_joint") {
        gripper_idx = i;
    } else if (info_.joints[i].name == "suction_joint") {
        suction_idx = i;
    }
  }

  // --- 夹爪状态直通通道 (Direct Pass-through) ---
  if (gripper_idx != -1 && gripper_status_pub_) {
      double stm32_gripper_status = joint_positions_[gripper_idx];

      if (std::abs(stm32_gripper_status - last_published_gripper_status_) > 0.01) {
          std_msgs::msg::Float64 status_msg;
          status_msg.data = stm32_gripper_status;
          gripper_status_pub_->publish(status_msg);
          last_published_gripper_status_ = stm32_gripper_status;

          RCLCPP_INFO(logger_, "[Hardware] 接收到 STM32 夹爪状态更新: %.1f", stm32_gripper_status);
      }
  }

  if (suction_idx != -1 && suction_status_pub_) {
      double stm32_suction_status = joint_positions_[suction_idx];
      if (std::abs(stm32_suction_status - last_published_suction_status_) > 0.01) {
          std_msgs::msg::Float64 suction_msg;
          suction_msg.data = stm32_suction_status;
          suction_status_pub_->publish(suction_msg);
          last_published_suction_status_ = stm32_suction_status;
          RCLCPP_INFO(logger_, "[Hardware] 接收到 STM32 吸盘状态更新: %.1f", stm32_suction_status);
      }
  }

  // 使用滤波后的位置计算速度，增加数据有效性检查
  if (last_read_time_.nanoseconds() != 0 && last_valid_time_.nanoseconds() != 0) {
    double dt = (last_valid_time_ - last_read_time_).seconds();
    if (dt > 0.002) {
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        double vel = (filtered_positions_[i] - last_positions_[i]) / dt;
        joint_velocities_[i] = std::clamp(vel, -10.0, 10.0);
      }
      last_positions_ = filtered_positions_;
      last_read_time_ = last_valid_time_;
    }
  } else if (frame_parsed) {
    last_positions_ = filtered_positions_;
    last_read_time_ = last_valid_time_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<uint8_t> buffer;
  buffer.push_back(0xAA);
  buffer.push_back(0x55);

  uint8_t active_joint_count = 0;
  for (const auto & joint : info_.joints) {
    if (!joint.command_interfaces.empty()) active_joint_count++;
  }
  buffer.push_back(active_joint_count);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!info_.joints[i].command_interfaces.empty()) {
      buffer.push_back(static_cast<uint8_t>(motor_ids_[i]));
      
      float angle = static_cast<float>(joint_commands_[i]);
      uint8_t angle_bytes[4];
      memcpy(angle_bytes, &angle, sizeof(float)); 
      buffer.insert(buffer.end(), angle_bytes, angle_bytes + 4);
    }
  }

  uint8_t checksum = 0;
  for (size_t i = 2; i < buffer.size(); ++i) checksum ^= buffer[i];
  buffer.push_back(checksum);

  if (!send_command(buffer)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Failed to send serial command");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool ArmHardwareInterface::open_serial(const std::string & port, int baudrate)
{
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ == -1) return false;

  struct termios options;
  tcgetattr(serial_fd_, &options);

  cfmakeraw(&options);
  speed_t baud = (baudrate == 115200) ? B115200 : B9600;
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
  options.c_cflag |= CS8;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 1;

  tcsetattr(serial_fd_, TCSANOW, &options);
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void ArmHardwareInterface::close_serial()
{
  if (serial_fd_ != -1) { close(serial_fd_); serial_fd_ = -1; }
}

bool ArmHardwareInterface::send_command(const std::vector<uint8_t> & data)
{
  if (serial_fd_ == -1) return false;
  int bytes_written = ::write(serial_fd_, data.data(), data.size());
  return (bytes_written == static_cast<int>(data.size()));
}

bool ArmHardwareInterface::parse_serial_buffer(std::vector<double> & positions)
{
  if (serial_buffer_.size() < 3) return false;

  const uint8_t header_pattern[] = {0xAA, 0x55};
  auto it = std::search(serial_buffer_.begin(), serial_buffer_.end(),
                        std::begin(header_pattern), std::end(header_pattern));

  if (it == serial_buffer_.end()) {
    if (serial_buffer_.size() > 100) {
      RCLCPP_WARN(logger_, "[Parse] 缓冲区堆积 %zu 字节但无帧头，清空", serial_buffer_.size());
      serial_buffer_.clear();
    }
    return false;
  }

  if (it != serial_buffer_.begin()) {
    RCLCPP_DEBUG(logger_, "[Parse] 丢弃帧头前 %ld 字节乱码", it - serial_buffer_.begin());
    serial_buffer_.erase(serial_buffer_.begin(), it);
  }

  if (serial_buffer_.size() < 3) return false;

  uint8_t joint_count = serial_buffer_[2];
  size_t expected_len = 3 + joint_count * sizeof(float) + 1;

  if (serial_buffer_.size() < expected_len) {
    RCLCPP_DEBUG(logger_, "[Parse] 缓冲区 %zu 字节不足一帧(%zu)，等待更多数据",
                 serial_buffer_.size(), expected_len);
    return false;
  }

  uint8_t received_checksum = serial_buffer_[expected_len - 1];
  uint8_t calculated_checksum = 0;
  for (size_t i = 2; i < expected_len - 1; ++i) calculated_checksum ^= serial_buffer_[i];

  if (received_checksum != calculated_checksum) {
    RCLCPP_WARN(logger_, "[Parse] 校验和错误 (received=0x%02X, calc=0x%02X)", received_checksum, calculated_checksum);
    auto next_hdr = std::search(serial_buffer_.begin() + 2, serial_buffer_.end(),
                                std::begin(header_pattern), std::end(header_pattern));
    if (next_hdr != serial_buffer_.end()) {
      serial_buffer_.erase(serial_buffer_.begin(), next_hdr);
    } else {
      serial_buffer_.erase(serial_buffer_.begin(), serial_buffer_.begin() + 2);
    }
    return false;
  }

  std::vector<size_t> active_indices;
  for (size_t k = 0; k < info_.joints.size(); ++k) {
      if (!info_.joints[k].command_interfaces.empty()) {
          active_indices.push_back(k);
      }
  }

  if (joint_count != active_indices.size()) {
    RCLCPP_WARN(logger_, "[Parse] 关节数量不匹配: STM32上报=%d, 本地配置=%zu",
                joint_count, active_indices.size());
  }

  size_t offset = 3;
  for (uint8_t i = 0; i < joint_count && i < active_indices.size(); ++i) {
    float angle;
    uint8_t temp_bytes[sizeof(float)];

    for (size_t j = 0; j < sizeof(float); ++j) {
      temp_bytes[j] = serial_buffer_[offset + j];
    }
    memcpy(&angle, temp_bytes, sizeof(float));

    size_t target_index = active_indices[i];
    positions[target_index] = static_cast<double>(angle);

    offset += sizeof(float);
  }

  serial_buffer_.erase(serial_buffer_.begin(), serial_buffer_.begin() + expected_len);
  RCLCPP_DEBUG(logger_, "[Parse] 成功解析一帧，关节数=%d", joint_count);
  return true;
}

std::vector<double> ArmHardwareInterface::read_joint_angles_from_serial(bool& frame_parsed)
{
  frame_parsed = false;
  std::vector<double> positions = joint_positions_;
  if (serial_fd_ == -1) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "[Serial] 串口未打开，无法读取");
    return positions;
  }

  uint8_t tmp[128];
  int bytes_read = ::read(serial_fd_, tmp, sizeof(tmp));

  if (bytes_read > 0) {
    RCLCPP_DEBUG(logger_, "[Serial] 收到 %d 字节原始数据", bytes_read);
    for (int i = 0; i < bytes_read; ++i) serial_buffer_.push_back(tmp[i]);
  } else if (bytes_read < 0) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "[Serial] 读取错误: %s", strerror(errno));
  }

  std::vector<double> parsed = positions;
  while (parse_serial_buffer(parsed)) {
    positions = parsed;
    frame_parsed = true;
  }
  return positions;
}

}  // namespace my_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_arm_hardware::ArmHardwareInterface,
  hardware_interface::SystemInterface)
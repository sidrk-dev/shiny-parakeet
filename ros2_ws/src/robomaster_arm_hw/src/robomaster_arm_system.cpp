#include "robomaster_arm_hw/robomaster_arm_system.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <termios.h>
#include <type_traits>
#include <unordered_map>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robomaster_arm_hw
{
namespace
{
constexpr const char * kLoggerName = "robomaster_arm_hw";

template <typename T>
T parse_or(const std::unordered_map<std::string, std::string> & params, const std::string & key, T fallback)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  if constexpr (std::is_integral<T>::value) {
    return static_cast<T>(std::stol(it->second));
  } else {
    return static_cast<T>(std::stod(it->second));
  }
}
}  // namespace

RoboMasterArmSystemHardware::~RoboMasterArmSystemHardware()
{
  close_serial();
}

hardware_interface::CallbackReturn RoboMasterArmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_count_ = info_.joints.size();
  if (joint_count_ == 0 || joint_count_ > kMaxJoints) {
    RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Expected 1 to %zu joints, got %zu", kMaxJoints, joint_count_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto serial_it = info_.hardware_parameters.find("serial_port");
  if (serial_it != info_.hardware_parameters.end()) {
    serial_port_ = serial_it->second;
  }
  baud_rate_ = parse_or<int>(info_.hardware_parameters, "baud_rate", baud_rate_);
  telemetry_timeout_ms_ = parse_or<int>(info_.hardware_parameters, "telemetry_timeout_ms", telemetry_timeout_ms_);
  max_consecutive_dropouts_ = parse_or<int>(info_.hardware_parameters, "max_consecutive_dropouts", max_consecutive_dropouts_);

  for (std::size_t i = 0; i < joint_count_; ++i) {
    const auto & joint = info_.joints[i];
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Joint %s must expose exactly one position command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Joint %s must expose position, velocity, and effort state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto motor_id_it = joint.parameters.find("motor_id");
    const int motor_id = motor_id_it == joint.parameters.end()
      ? static_cast<int>(i + 1)
      : std::stoi(motor_id_it->second);
    if (motor_id < 1 || motor_id > static_cast<int>(RoboMasterArmProtocol::kMaxMotors)) {
      RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Joint %s has invalid motor_id %d", joint.name.c_str(), motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    motor_ids_[i] = static_cast<uint8_t>(motor_id);
  }

  hw_positions_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_efforts_.fill(0.0);
  hw_position_commands_.fill(0.0);
  latest_telemetry_time_ = std::chrono::steady_clock::time_point{};

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboMasterArmSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboMasterArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  active_ = true;
  consecutive_dropouts_ = 0;
  fatal_logged_.store(false);
  send_telemetry_request();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboMasterArmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;

  RoboMasterArmProtocol::CommandPacket packet{};
  packet.host_time_ms = host_time_ms();
  packet.motor_count = RoboMasterArmProtocol::kMaxMotors;
  for (std::size_t i = 0; i < RoboMasterArmProtocol::kMaxMotors; ++i) {
    packet.mode[i] = static_cast<uint8_t>(RoboMasterArmProtocol::MotorMode::Stop);
  }
  send_packet(packet, RoboMasterArmProtocol::PacketType::Command);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboMasterArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_count_ * 3);
  for (std::size_t i = 0; i < joint_count_; ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RoboMasterArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_count_);
  for (std::size_t i = 0; i < joint_count_; ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type RoboMasterArmSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  send_telemetry_request();

  RoboMasterArmProtocol::TelemetryPacket telemetry{};
  std::chrono::steady_clock::time_point telemetry_time;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    telemetry = latest_telemetry_;
    telemetry_time = latest_telemetry_time_;
  }

  const auto now = std::chrono::steady_clock::now();
  if (telemetry_time == std::chrono::steady_clock::time_point{} ||
      std::chrono::duration_cast<std::chrono::milliseconds>(now - telemetry_time).count() > telemetry_timeout_ms_) {
    consecutive_dropouts_++;
    if (consecutive_dropouts_ > max_consecutive_dropouts_) {
      if (!fatal_logged_.exchange(true)) {
        RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "RP2350 telemetry timed out for %d consecutive control cycles", consecutive_dropouts_);
      }
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  consecutive_dropouts_ = 0;
  for (std::size_t i = 0; i < joint_count_; ++i) {
    const std::size_t motor_index = static_cast<std::size_t>(motor_ids_[i] - 1);
    hw_positions_[i] = static_cast<double>(telemetry.position_deg[motor_index]) * kDegToRad;
    hw_velocities_[i] = static_cast<double>(telemetry.velocity_rpm[motor_index]) * kRpmToRadPerSec;
    hw_efforts_[i] = static_cast<double>(telemetry.current_ma[motor_index]) / 1000.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboMasterArmSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_ || !serial_connected_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  RoboMasterArmProtocol::CommandPacket packet{};
  packet.host_time_ms = host_time_ms();
  packet.motor_count = RoboMasterArmProtocol::kMaxMotors;
  for (std::size_t i = 0; i < RoboMasterArmProtocol::kMaxMotors; ++i) {
    packet.mode[i] = static_cast<uint8_t>(RoboMasterArmProtocol::MotorMode::Stop);
  }

  for (std::size_t i = 0; i < joint_count_; ++i) {
    const std::size_t motor_index = static_cast<std::size_t>(motor_ids_[i] - 1);
    packet.mode[motor_index] = static_cast<uint8_t>(RoboMasterArmProtocol::MotorMode::Position);
    packet.target_position_deg[motor_index] = static_cast<float>(hw_position_commands_[i] * kRadToDeg);
  }

  return send_packet(packet, RoboMasterArmProtocol::PacketType::Command)
    ? hardware_interface::return_type::OK
    : hardware_interface::return_type::ERROR;
}

bool RoboMasterArmSystemHardware::open_serial()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Failed to open %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  if (!configure_serial_port(serial_fd_, baud_rate_)) {
    close_serial();
    return false;
  }

  serial_connected_.store(true);
  reader_running_.store(true);
  reader_thread_ = std::thread(&RoboMasterArmSystemHardware::reader_loop, this);
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Opened RP2350 serial port %s at %d baud", serial_port_.c_str(), baud_rate_);
  return true;
}

void RoboMasterArmSystemHardware::close_serial()
{
  reader_running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  serial_connected_.store(false);
}

bool RoboMasterArmSystemHardware::configure_serial_port(int fd, int baud_rate)
{
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "tcgetattr failed: %s", std::strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  const int baud_constant = baud_to_constant(baud_rate);
  if (baud_constant < 0) {
    RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Unsupported baud rate %d", baud_rate);
    return false;
  }
  cfsetispeed(&tty, static_cast<speed_t>(baud_constant));
  cfsetospeed(&tty, static_cast<speed_t>(baud_constant));
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "tcsetattr failed: %s", std::strerror(errno));
    return false;
  }
  tcflush(fd, TCIOFLUSH);
  return true;
}

int RoboMasterArmSystemHardware::baud_to_constant(int baud_rate) const
{
  switch (baud_rate) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return -1;
  }
}

bool RoboMasterArmSystemHardware::write_bytes(const uint8_t * data, std::size_t length)
{
  if (serial_fd_ < 0 || !serial_connected_.load()) {
    return false;
  }
  if (!write_mutex_.try_lock()) {
    return false;
  }

  std::size_t written = 0;
  while (written < length) {
    const ssize_t rc = ::write(serial_fd_, data + written, length - written);
    if (rc > 0) {
      written += static_cast<std::size_t>(rc);
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      write_mutex_.unlock();
      return false;
    }
    write_mutex_.unlock();
    serial_connected_.store(false);
    return false;
  }

  const uint8_t delimiter = 0;
  const ssize_t rc = ::write(serial_fd_, &delimiter, 1);
  write_mutex_.unlock();
  if (rc != 1) {
    serial_connected_.store(false);
    return false;
  }
  return true;
}

template <typename PacketT>
bool RoboMasterArmSystemHardware::send_packet(PacketT & packet, RoboMasterArmProtocol::PacketType type)
{
  RoboMasterArmProtocol::finalizePacket(packet, type, tx_sequence_++);
  const std::size_t encoded_length = RoboMasterArmProtocol::cobsEncode(
    reinterpret_cast<const uint8_t *>(&packet),
    sizeof(PacketT),
    tx_encoded_.data(),
    tx_encoded_.size());
  if (encoded_length == 0) {
    return false;
  }
  return write_bytes(tx_encoded_.data(), encoded_length);
}

bool RoboMasterArmSystemHardware::send_telemetry_request()
{
  RoboMasterArmProtocol::TelemetryRequestPacket packet{};
  packet.host_time_ms = host_time_ms();
  return send_packet(packet, RoboMasterArmProtocol::PacketType::TelemetryRequest);
}

void RoboMasterArmSystemHardware::reader_loop()
{
  rx_encoded_length_ = 0;
  while (reader_running_.load()) {
    uint8_t byte = 0;
    const ssize_t rc = ::read(serial_fd_, &byte, 1);
    if (rc == 1) {
      if (byte == 0) {
        if (rx_encoded_length_ > 0) {
          const std::size_t decoded_length = RoboMasterArmProtocol::cobsDecode(
            rx_encoded_.data(),
            rx_encoded_length_,
            rx_decoded_.data(),
            rx_decoded_.size());
          if (decoded_length > 0) {
            handle_frame(rx_decoded_.data(), decoded_length);
          }
          rx_encoded_length_ = 0;
        }
      } else if (rx_encoded_length_ < rx_encoded_.size()) {
        rx_encoded_[rx_encoded_length_++] = byte;
      } else {
        rx_encoded_length_ = 0;
      }
      continue;
    }

    if (rc < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      serial_connected_.store(false);
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void RoboMasterArmSystemHardware::handle_frame(const uint8_t * data, std::size_t length)
{
  if (length != sizeof(RoboMasterArmProtocol::TelemetryPacket)) {
    return;
  }

  RoboMasterArmProtocol::TelemetryPacket packet{};
  std::memcpy(&packet, data, sizeof(packet));
  if (!RoboMasterArmProtocol::validatePacket(packet, RoboMasterArmProtocol::PacketType::Telemetry)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    latest_telemetry_ = packet;
    latest_telemetry_time_ = std::chrono::steady_clock::now();
  }
  telemetry_counter_.fetch_add(1, std::memory_order_relaxed);
}

uint32_t RoboMasterArmSystemHardware::host_time_ms() const
{
  using namespace std::chrono;
  const auto now = steady_clock::now().time_since_epoch();
  return static_cast<uint32_t>(duration_cast<milliseconds>(now).count());
}

}  // namespace robomaster_arm_hw

PLUGINLIB_EXPORT_CLASS(
  robomaster_arm_hw::RoboMasterArmSystemHardware,
  hardware_interface::SystemInterface)

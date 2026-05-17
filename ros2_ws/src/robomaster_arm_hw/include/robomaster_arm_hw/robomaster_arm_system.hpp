#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robomaster_arm_hw/protocol.hpp"

namespace robomaster_arm_hw
{

class RoboMasterArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoboMasterArmSystemHardware)

  ~RoboMasterArmSystemHardware() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kMaxJoints = RoboMasterArmProtocol::kMaxMotors;
  static constexpr double kDegToRad = 0.017453292519943295769;
  static constexpr double kRpmToRadPerSec = 0.10471975511965977462;
  static constexpr double kRadToDeg = 57.2957795130823208768;

  bool open_serial();
  void close_serial();
  bool configure_serial_port(int fd, int baud_rate);
  bool write_bytes(const uint8_t * data, std::size_t length);

  template <typename PacketT>
  bool send_packet(PacketT & packet, RoboMasterArmProtocol::PacketType type);

  bool send_telemetry_request();
  void reader_loop();
  void handle_frame(const uint8_t * data, std::size_t length);

  int baud_to_constant(int baud_rate) const;
  uint32_t host_time_ms() const;

  std::string serial_port_ = "/dev/ttyACM0";
  int baud_rate_ = 921600;
  int serial_fd_ = -1;

  std::thread reader_thread_;
  std::atomic_bool reader_running_{false};
  std::atomic_bool serial_connected_{false};
  std::atomic_bool fatal_logged_{false};

  std::mutex write_mutex_;
  std::mutex telemetry_mutex_;
  RoboMasterArmProtocol::TelemetryPacket latest_telemetry_{};
  std::chrono::steady_clock::time_point latest_telemetry_time_{};
  std::atomic<uint32_t> telemetry_counter_{0};

  std::array<uint8_t, RoboMasterArmProtocol::kMaxEncodedBytes> tx_encoded_{};
  std::array<uint8_t, RoboMasterArmProtocol::kMaxEncodedBytes> rx_encoded_{};
  std::array<uint8_t, RoboMasterArmProtocol::kMaxPacketBytes> rx_decoded_{};
  std::size_t rx_encoded_length_ = 0;
  uint32_t tx_sequence_ = 1;

  std::size_t joint_count_ = 0;
  std::array<uint8_t, kMaxJoints> motor_ids_{};
  std::array<double, kMaxJoints> hw_positions_{};
  std::array<double, kMaxJoints> hw_velocities_{};
  std::array<double, kMaxJoints> hw_efforts_{};
  std::array<double, kMaxJoints> hw_position_commands_{};

  int consecutive_dropouts_ = 0;
  int max_consecutive_dropouts_ = 25;
  int telemetry_timeout_ms_ = 250;
  bool active_ = false;
};

}  // namespace robomaster_arm_hw

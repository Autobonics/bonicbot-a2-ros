#ifndef MY_BOT__DIFFBOT_SYSTEM_HPP_
#define MY_BOT__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "my_bot/uart_protocol.hpp"

// Serial communication includes
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace my_bot
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  // Test comment
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial port management
  bool openSerialPort();
  void clearBuffer();
  void closeSerialPort();
  
  // Binary packet communication
  bool sendPacket(uint8_t packet_type, const uint8_t* payload, uint8_t length);
  void receivePacket();
  void processPacket(uint8_t packet_type, const uint8_t* payload, uint8_t length);
  
  // Servo control methods
  void initializeServoMapping();
  bool sendServoControl(uint8_t servo_id, double position_rad, int16_t speed, uint8_t acceleration);
  bool sendMultipleServoControl(const std::vector<std::tuple<uint8_t, double, int16_t, uint8_t>>& commands);
  bool requestServoFeedback(const std::vector<uint8_t>& servo_ids);
  void processServoFeedback(const uint8_t* payload, uint8_t length);
  
  // Robot parameters from URDF
  std::string serial_port_;
  int serial_baud_;
  double encoder_cpr_;
  double wheel_radius_;
  double max_velocity_;
  
  // Communication
  int serial_fd_ = -1;
  bool connected_ = false;
  
  // Packet receive state machine
  uart_protocol::RxState rx_state_ = uart_protocol::RxState::WAIT_HEADER1;
  uint8_t rx_packet_type_ = 0;
  uint8_t rx_length_ = 0;
  uint8_t rx_payload_[uart_protocol::MAX_PAYLOAD_SIZE];
  uint8_t rx_payload_index_ = 0;
  uint8_t rx_checksum_ = 0;
  
  // Joint states
  std::vector<double> hw_commands_;      // Commanded velocities (rad/s)
  std::vector<double> hw_positions_;     // Position (rad)
  std::vector<double> hw_velocities_;    // Velocity (rad/s)
  std::vector<double> prev_positions_;   // For velocity calculation
  
  // Encoder data (received from ESP32)
  int32_t encoder_left_ = 0;
  int32_t encoder_right_ = 0;
  bool encoder_data_ready_ = false;
  
  // Servo mapping and state
  std::map<std::string, uint8_t> joint_to_servo_id_;  // Joint name -> Servo ID
  std::map<uint8_t, double> servo_positions_;         // Servo ID -> Position (rad)
  std::map<uint8_t, bool> servo_feedback_ready_;      // Servo ID -> Feedback ready flag
  std::vector<uint8_t> servo_joint_indices_;          // Indices of servo joints in hw_* vectors
  
  // Servo command optimization
  std::vector<double> last_servo_commands_;           // Last commanded positions
  int servo_update_counter_ = 0;                      // Counter for rate limiting
  const double servo_position_threshold_ = 0.01;      // Min change to send command (rad ~0.57 deg)
};

}  // namespace my_bot

#endif  // MY_BOT__DIFFBOT_SYSTEM_HPP_
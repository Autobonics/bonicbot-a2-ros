#include "my_bot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>
#include <cerrno>
#include <cstring>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_bot
{

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  serial_port_ = info_.hardware_parameters["serial_port"];
  serial_baud_ = std::stoi(info_.hardware_parameters["serial_baud"]);
  encoder_cpr_ = std::stod(info_.hardware_parameters["encoder_cpr"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Serial Port: %s, Baud: %d", serial_port_.c_str(), serial_baud_);
  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Encoder CPR: %.0f, Wheel radius: %.3f m, Max velocity: %.2f rad/s",
    encoder_cpr_, wheel_radius_, max_velocity_);

  // Initialize vectors
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  prev_positions_.resize(info_.joints.size(), 0.0);

  // Verify joint configuration and identify joint types
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces. 1 expected.", 
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string& cmd_type = joint.command_interfaces[0].name;
    
    // Support both velocity (wheels) and position (servos) command interfaces
    if (cmd_type != hardware_interface::HW_IF_VELOCITY && 
        cmd_type != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has unsupported command interface '%s'. Expected velocity or position.", 
        joint.name.c_str(), cmd_type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interfaces. 2 expected.", 
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Track servo joints (position command interface)
    if (cmd_type == hardware_interface::HW_IF_POSITION)
    {
      servo_joint_indices_.push_back(i);
      RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Servo joint '%s' at index %zu", joint.name.c_str(), i);
    }
  }

  // Initialize servo mapping
  initializeServoMapping();
  
  // Initialize last servo command tracking
  last_servo_commands_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"),
    "Initialized %zu joints (%zu wheels, %zu servos)",
    info_.joints.size(), 
    info_.joints.size() - servo_joint_indices_.size(),
    servo_joint_indices_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // Determine command interface type based on joint configuration
    const std::string& cmd_type = info_.joints[i].command_interfaces[0].name;
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, cmd_type, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Configuring...");

  // Open serial port connection to ESP32
  if (!openSerialPort())
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                 "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), 
              "Successfully opened serial port!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating...");

  // Clear any buffered data
  clearBuffer();
  
  // Give ESP32 time to be ready
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Reset encoders on ESP32
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Resetting encoders...");
  
  for (int attempt = 0; attempt < 3; attempt++)
  {
    clearBuffer();
    
    if (!sendPacket(uart_protocol::CMD_RESET_ENC, nullptr, 0))
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                   "Failed to send reset command (attempt %d)", attempt + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // Wait for ACK response
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Try to receive response packets
    for (int i = 0; i < 50; i++)  // Check for ~250ms
    {
      receivePacket();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Encoders reset sent");
    break;
  }

  // Test encoder read
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Testing encoder read...");
  
  for (int attempt = 0; attempt < 3; attempt++)
  {
    clearBuffer();
    encoder_data_ready_ = false;
    
    if (!sendPacket(uart_protocol::CMD_ENCODER_REQ, nullptr, 0))
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                   "Failed to send encoder request (attempt %d)", attempt + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // Wait for response and process packets
    auto start_time = std::chrono::steady_clock::now();
    while (!encoder_data_ready_)
    {
      receivePacket();
      
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();
      
      if (elapsed > 500)  // 500ms timeout
        break;
        
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    if (encoder_data_ready_)
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), 
                  "Successfully read encoders: %d, %d", encoder_left_, encoder_right_);
      break;
    }
    
    if (attempt == 2)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                   "Failed to read encoders from ESP32 after 3 attempts");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Initialize positions
  for (uint i = 0; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
    prev_positions_[i] = 0.0;
  }

  // Encoder values are initialized in class definition
  encoder_left_ = 0;
  encoder_right_ = 0;
  encoder_data_ready_ = false;

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating...");

  // Stop motors
  sendPacket(uart_protocol::CMD_STOP, nullptr, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  static int error_count = 0;
  
  if (!connected_)
  {
    if (error_count++ % 100 == 0) // Log every 100th error
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Not connected to ESP32");
    }
    return hardware_interface::return_type::ERROR;
  }

  // ========== READ ENCODERS (Wheels) ==========
  encoder_data_ready_ = false;
  
  if (!sendPacket(uart_protocol::CMD_ENCODER_REQ, nullptr, 0))
  {
    if (error_count++ % 100 == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Failed to send encoder request");
    }
    return hardware_interface::return_type::ERROR;
  }

  // Process incoming packets (non-blocking)
  // Try multiple times to give ESP32 time to respond
  auto start_time = std::chrono::steady_clock::now();
  while (!encoder_data_ready_)
  {
    receivePacket();
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();
    
    if (elapsed > 100)  // 100ms timeout
      break;
      
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  
  if (!encoder_data_ready_)
  {
    if (error_count++ % 100 == 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), "No encoder response from ESP32");
    }
    // Don't fail, use last known values and continue to servos
  }
  else
  {
    // Convert encoder ticks to radians (absolute position)
    hw_positions_[0] = (encoder_left_ / encoder_cpr_) * 2.0 * M_PI;
    hw_positions_[1] = (encoder_right_ / encoder_cpr_) * 2.0 * M_PI;
  }


  // ========== READ SERVO FEEDBACK at 2 Hz (every 20 cycles) ==========
  // Request servo feedback every 20 cycles to match 2 Hz servo command rate
  if (!servo_joint_indices_.empty() && servo_update_counter_ == 0)
  {
    std::vector<uint8_t> servo_ids;
    for (size_t joint_idx : servo_joint_indices_)
    {
      const std::string& joint_name = info_.joints[joint_idx].name;
      auto it = joint_to_servo_id_.find(joint_name);
      if (it != joint_to_servo_id_.end())
      {
        servo_ids.push_back(it->second);
        servo_feedback_ready_[it->second] = false;
      }
    }
    
    if (!servo_ids.empty() && requestServoFeedback(servo_ids))
    {
      start_time = std::chrono::steady_clock::now();
      bool any_feedback = false;
      
      while (true)
      {
        receivePacket();
        
        for (uint8_t servo_id : servo_ids)
        {
          if (servo_feedback_ready_[servo_id])
          {
            any_feedback = true;
          }
        }
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_time).count();
        
        if (elapsed > 100 || any_feedback)
          break;
          
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      
      if (!any_feedback && error_count++ % 100 == 0)
      {
        RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                    "No servo feedback received");
      }
    }
  }

  // Reset error counter on success
  error_count = 0;

  // Calculate velocities (rad/s) for all joints
  double dt = period.seconds();
  if (dt > 0.0)
  {
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
      hw_velocities_[i] = (hw_positions_[i] - prev_positions_[i]) / dt;
      prev_positions_[i] = hw_positions_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  static int error_count = 0;
  
  if (!connected_)
  {
    if (error_count++ % 100 == 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Not connected to ESP32");
    }
    return hardware_interface::return_type::ERROR;
  }


  // ========== WHEEL MOTORS at 20 Hz (every 2 cycles) ==========
  static int motor_counter = 0;
  motor_counter++;
  
  if (motor_counter >= 2)  // 40 Hz / 2 = 20 Hz
  {
    motor_counter = 0;
    
    // Convert commanded velocities (rad/s) to PWM values
    int16_t pwm_left = static_cast<int16_t>((hw_commands_[0] / max_velocity_) * 1022.0);
    int16_t pwm_right = static_cast<int16_t>((hw_commands_[1] / max_velocity_) * 1022.0);

    // Clamp PWM values to valid range (-1022 to 1022)
    pwm_left = std::max<int16_t>(-1022, std::min<int16_t>(1022, pwm_left));
    pwm_right = std::max<int16_t>(-1022, std::min<int16_t>(1022, pwm_right));

    // Build payload: two int16_t values (4 bytes total, little-endian)
    uint8_t payload[4];
    payload[0] = pwm_left & 0xFF;
    payload[1] = (pwm_left >> 8) & 0xFF;
    payload[2] = pwm_right & 0xFF;
    payload[3] = (pwm_right >> 8) & 0xFF;

    if (!sendPacket(uart_protocol::CMD_PWM, payload, 4))
    {
      if (error_count++ % 100 == 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Failed to send motor command");
      }
      return hardware_interface::return_type::ERROR;
    }
  }


  // ========== SERVOS at 2 Hz (every 20 cycles) with change detection ==========
  servo_update_counter_++;
  bool servo_update_cycle = (servo_update_counter_ >= 20);  // 40 Hz / 20 = 2 Hz
  
  if (servo_update_cycle)
    servo_update_counter_ = 0;
  
  // Only send commands when servos actually move
  if (servo_update_cycle)
  {
    std::vector<std::tuple<uint8_t, double, int16_t, uint8_t>> servo_commands;
    
    for (size_t joint_idx : servo_joint_indices_)
    {
      const std::string& joint_name = info_.joints[joint_idx].name;
      auto it = joint_to_servo_id_.find(joint_name);
      
      if (it != joint_to_servo_id_.end())
      {
        uint8_t servo_id = it->second;
        double position_cmd = hw_commands_[joint_idx];
        
        // Only send if position changed significantly
        bool position_changed = std::abs(position_cmd - last_servo_commands_[joint_idx]) > servo_position_threshold_;
        
        if (position_changed)
        {
          int16_t speed = 500;
          uint8_t acceleration = 50;
          
          servo_commands.push_back(std::make_tuple(servo_id, position_cmd, speed, acceleration));
          last_servo_commands_[joint_idx] = position_cmd;
        }
      }
    }
    
    // Send batched commands only if servos are moving
    if (!servo_commands.empty())
    {
      if (!sendMultipleServoControl(servo_commands))
      {
        if (error_count++ % 100 == 0)
        {
          RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                      "Failed to send servo commands (%zu servos)", servo_commands.size());
        }
      }
    }
  }

  // Reset error counter on success
  error_count = 0;

  return hardware_interface::return_type::OK;
}

// ==================== SERIAL COMMUNICATION FUNCTIONS ====================

bool DiffBotSystemHardware::openSerialPort()
{
  // Open serial port
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                 "Failed to open serial port %s: %s", serial_port_.c_str(), strerror(errno));
    return false;
  }

  // Configure serial port
  struct termios options;
  if (tcgetattr(serial_fd_, &options) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                 "Failed to get serial port attributes: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Set baud rate
  speed_t baud;
  switch (serial_baud_)
  {
    case 9600:   baud = B9600; break;
    case 19200:  baud = B19200; break;
    case 38400:  baud = B38400; break;
    case 57600:  baud = B57600; break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                   "Unsupported baud rate: %d", serial_baud_);
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
  }

  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  // 8N1 mode
  options.c_cflag &= ~PARENB;        // No parity
  options.c_cflag &= ~CSTOPB;        // 1 stop bit
  options.c_cflag &= ~CSIZE;         // Clear data size bits
  options.c_cflag |= CS8;            // 8 data bits
  options.c_cflag &= ~CRTSCTS;       // No hardware flow control
  options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem lines

  // Raw input mode
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
  options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Raw output mode
  options.c_oflag &= ~OPOST;

  // Non-blocking reads with timeout
  options.c_cc[VMIN] = 0;   // Don't wait for specific number of bytes
  options.c_cc[VTIME] = 1;  // 0.1 second timeout

  // Apply settings
  tcflush(serial_fd_, TCIFLUSH);
  if (tcsetattr(serial_fd_, TCSANOW, &options) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                 "Failed to set serial port attributes: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Wait for port to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Clear any initial garbage data
  clearBuffer();

  connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), 
              "Serial port opened: %s at %d baud", serial_port_.c_str(), serial_baud_);
  
  return true;
}

bool DiffBotSystemHardware::sendPacket(uint8_t packet_type, const uint8_t* payload, uint8_t length)
{
  if (serial_fd_ < 0 || !connected_)
    return false;

  // Validate payload length
  if (length > uart_protocol::MAX_PAYLOAD_SIZE)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                 "Payload too large: %d bytes (max %d)", length, uart_protocol::MAX_PAYLOAD_SIZE);
    return false;
  }

  // Build packet: [HEADER1] [HEADER2] [TYPE] [LENGTH] [PAYLOAD...] [CHECKSUM]
  uint8_t packet[4 + uart_protocol::MAX_PAYLOAD_SIZE + 1];
  uint8_t packet_size = 0;
  
  packet[packet_size++] = uart_protocol::HEADER_BYTE1;
  packet[packet_size++] = uart_protocol::HEADER_BYTE2;
  packet[packet_size++] = packet_type;
  packet[packet_size++] = length;
  
  // Copy payload if present
  if (length > 0 && payload != nullptr)
  {
    for (uint8_t i = 0; i < length; i++)
    {
      packet[packet_size++] = payload[i];
    }
  }
  
  // Calculate and append checksum
  uint8_t checksum = uart_protocol::calculate_checksum(packet_type, length, payload);
  packet[packet_size++] = checksum;
  
  // Send packet
  ssize_t bytes_written = ::write(serial_fd_, packet, packet_size);
  
  if (bytes_written < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), 
                 "Failed to write packet: %s", strerror(errno));
    return false;
  }

  if (static_cast<uint8_t>(bytes_written) != packet_size)
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                "Incomplete packet write: %zd of %d bytes", bytes_written, packet_size);
  }

  // Ensure data is sent immediately
  tcdrain(serial_fd_);

  return true;
}

void DiffBotSystemHardware::receivePacket()
{
  if (serial_fd_ < 0 || !connected_)
    return;

  // Check if data is available
  int bytes_available = 0;
  if (ioctl(serial_fd_, FIONREAD, &bytes_available) < 0)
    return;
  
  if (bytes_available <= 0)
    return;
  
  // Read available bytes one at a time for state machine
  uint8_t byte;
  ssize_t bytes_read = ::read(serial_fd_, &byte, 1);
  
  if (bytes_read != 1)
    return;
  
  // State machine for packet parsing
  switch (rx_state_)
  {
    case uart_protocol::RxState::WAIT_HEADER1:
      if (byte == uart_protocol::HEADER_BYTE1)
      {
        rx_state_ = uart_protocol::RxState::WAIT_HEADER2;
      }
      break;
      
    case uart_protocol::RxState::WAIT_HEADER2:
      if (byte == uart_protocol::HEADER_BYTE2)
      {
        rx_state_ = uart_protocol::RxState::WAIT_TYPE;
      }
      else if (byte == uart_protocol::HEADER_BYTE1)
      {
        // Stay in WAIT_HEADER2 (could be start of new packet)
        rx_state_ = uart_protocol::RxState::WAIT_HEADER2;
      }
      else
      {
        rx_state_ = uart_protocol::RxState::WAIT_HEADER1;
      }
      break;
      
    case uart_protocol::RxState::WAIT_TYPE:
      rx_packet_type_ = byte;
      rx_state_ = uart_protocol::RxState::WAIT_LENGTH;
      break;
      
    case uart_protocol::RxState::WAIT_LENGTH:
      rx_length_ = byte;
      rx_payload_index_ = 0;
      
      if (rx_length_ == 0)
      {
        rx_state_ = uart_protocol::RxState::WAIT_CHECKSUM;
      }
      else if (rx_length_ > uart_protocol::MAX_PAYLOAD_SIZE)
      {
        // Invalid length, reset
        rx_state_ = uart_protocol::RxState::WAIT_HEADER1;
      }
      else
      {
        rx_state_ = uart_protocol::RxState::WAIT_PAYLOAD;
      }
      break;
      
    case uart_protocol::RxState::WAIT_PAYLOAD:
      rx_payload_[rx_payload_index_++] = byte;
      
      if (rx_payload_index_ >= rx_length_)
      {
        rx_state_ = uart_protocol::RxState::WAIT_CHECKSUM;
      }
      break;
      
    case uart_protocol::RxState::WAIT_CHECKSUM:
      rx_checksum_ = byte;
      
      // Validate checksum
      uint8_t calculated_checksum = uart_protocol::calculate_checksum(
        rx_packet_type_, rx_length_, rx_payload_);
      
      if (rx_checksum_ == calculated_checksum)
      {
        // Valid packet, process it
        processPacket(rx_packet_type_, rx_payload_, rx_length_);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                    "Checksum mismatch: expected 0x%02X, got 0x%02X", 
                    calculated_checksum, rx_checksum_);
      }
      
      // Reset state machine for next packet
      rx_state_ = uart_protocol::RxState::WAIT_HEADER1;
      break;
  }
}

void DiffBotSystemHardware::processPacket(uint8_t packet_type, const uint8_t* payload, uint8_t length)
{
  switch (packet_type)
  {
    case uart_protocol::RESP_ENCODER:
      if (length == 8)
      {
        // Parse two int32_t values (little-endian)
        encoder_left_ = static_cast<int32_t>(
          payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));
        encoder_right_ = static_cast<int32_t>(
          payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24));
        encoder_data_ready_ = true;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                    "Invalid encoder response length: %d (expected 8)", length);
      }
      break;
      
    case uart_protocol::RESP_ACK:
      // Acknowledgment received
      break;
      
    case uart_protocol::RESP_ERROR:
      RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                  "Error response from ESP32");
      break;
      
    case uart_protocol::RESP_FEEDBACK:
      // Old-style single servo feedback (for backward compatibility)
      break;
      
    case uart_protocol::RESP_SERVO_FEEDBACK:
      // Process servo feedback from multiple servos
      processServoFeedback(payload, length);
      break;
      
    default:
      RCLCPP_DEBUG(rclcpp::get_logger("DiffBotSystemHardware"), 
                   "Unknown packet type: 0x%02X", packet_type);
      break;
  }
}

void DiffBotSystemHardware::clearBuffer()
{
  if (serial_fd_ < 0 || !connected_)
    return;

  tcflush(serial_fd_, TCIFLUSH);
  
  // Also read and discard any buffered data
  char buffer[256];
  while (true)
  {
    int bytes_available = 0;
    if (ioctl(serial_fd_, FIONREAD, &bytes_available) < 0)
      break;
      
    if (bytes_available <= 0)
      break;
      
    ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
    if (bytes_read <= 0)
      break;
  }
}

void DiffBotSystemHardware::closeSerialPort()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Serial port closed");
  }
  connected_ = false;
}

// ==================== SERVO CONTROL FUNCTIONS ====================

void DiffBotSystemHardware::initializeServoMapping()
{
  // Mapping from joint names to servo IDs (1-14 in firmware, user-facing)
  // Based on user-provided mapping:
  // right gripper - 1, right elbow - 3, right pitch - 6
  // left gripper - 7, left elbow - 9, left pitch - 12, neck - 13
  
  joint_to_servo_id_["right_gripper_finger1_joint"] = 1;
  joint_to_servo_id_["right_elbow_joint"] = 3;
  joint_to_servo_id_["right_shoulder_pitch_joint"] = 6;
  joint_to_servo_id_["left_gripper_finger1_joint"] = 7;
  joint_to_servo_id_["left_elbow_joint"] = 9;
  joint_to_servo_id_["left_shoulder_pitch_joint"] = 12;
  joint_to_servo_id_["neck_yaw_joint"] = 13;
  
  // Initialize servo state maps
  for (const auto& pair : joint_to_servo_id_)
  {
    uint8_t servo_id = pair.second;
    servo_positions_[servo_id] = 0.0;
    servo_feedback_ready_[servo_id] = false;
    
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "Mapped joint '%s' to servo ID %d", pair.first.c_str(), servo_id);
  }
}

bool DiffBotSystemHardware::sendServoControl(
  uint8_t servo_id, double position_rad, int16_t speed, uint8_t acceleration)
{
  if (serial_fd_ < 0 || !connected_)
    return false;

  // Convert position from radians to degrees for servo
  float angle_deg = position_rad * (180.0 / M_PI);
  
  // Build payload: count(1) + [id(1) + angle(4) + speed(2) + acc(1)] = 9 bytes total
  uint8_t payload[9];
  uint8_t idx = 0;
  
  // Count: 1 servo
  payload[idx++] = 1;
  
  // Servo data: ID (convert to firmware ID: 0-13)
  uint8_t firmware_id = (servo_id - 1) & 0xFF;
  payload[idx++] = firmware_id;
  
  // Angle (float, little-endian)
  memcpy(&payload[idx], &angle_deg, sizeof(float));
  idx += sizeof(float);
  
  // Speed (int16_t, little-endian)
  payload[idx++] = speed & 0xFF;
  payload[idx++] = (speed >> 8) & 0xFF;
  
  // Acceleration (uint8_t)
  payload[idx++] = acceleration;
  
  return sendPacket(uart_protocol::CMD_SERVO_CONTROL, payload, idx);
}

bool DiffBotSystemHardware::sendMultipleServoControl(
  const std::vector<std::tuple<uint8_t, double, int16_t, uint8_t>>& commands)
{
  if (serial_fd_ < 0 || !connected_ || commands.empty())
    return false;

  if (commands.size() > 14)
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                "Too many servo commands: %zu (max 14)", commands.size());
    return false;
  }

  // Build payload: count(1) + [id(1) + angle(4) + speed(2) + acc(1)] * count
  uint8_t payload[128];
  uint8_t idx = 0;
  
  // Count
  payload[idx++] = commands.size();
  
  // Pack each servo command
  for (const auto& cmd : commands)
  {
    uint8_t servo_id = std::get<0>(cmd);
    double position_rad = std::get<1>(cmd);
    int16_t speed = std::get<2>(cmd);
    uint8_t acceleration = std::get<3>(cmd);
    
    // Negate angles for elbows and grippers to match physical robot
    // Servo IDs: 1 (right gripper), 3 (right elbow), 7 (left gripper), 9 (left elbow)
    if (servo_id == 1 || servo_id == 3 || servo_id == 7 || servo_id == 9 || servo_id == 13)
    {
      position_rad = -position_rad;
    }
    
    // Convert position from radians to degrees for servo
    float angle_deg = position_rad * (180.0 / M_PI);
    
    // Servo ID (convert to firmware ID: 0-13)
    uint8_t firmware_id = (servo_id - 1) & 0xFF;
    payload[idx++] = firmware_id;
    
    // Angle (float, little-endian)
    memcpy(&payload[idx], &angle_deg, sizeof(float));
    idx += sizeof(float);
    
    // Speed (int16_t, little-endian)
    payload[idx++] = speed & 0xFF;
    payload[idx++] = (speed >> 8) & 0xFF;
    
    // Acceleration (uint8_t)
    payload[idx++] = acceleration;
  }
  
  return sendPacket(uart_protocol::CMD_SERVO_CONTROL, payload, idx);
}

bool DiffBotSystemHardware::requestServoFeedback(const std::vector<uint8_t>& servo_ids)
{
  if (serial_fd_ < 0 || !connected_)
    return false;

  uint8_t payload[128];
  uint8_t idx = 0;
  
  if (servo_ids.empty())
  {
    // Request all servos
    payload[idx++] = 0;
  }
  else
  {
    // Request specific servos
    payload[idx++] = servo_ids.size();
    for (uint8_t servo_id : servo_ids)
    {
      // Convert to firmware ID (0-13)
      uint8_t firmware_id = (servo_id - 1) & 0xFF;
      payload[idx++] = firmware_id;
    }
  }
  
  return sendPacket(uart_protocol::CMD_SERVO_FEEDBACK_REQ, payload, idx);
}

void DiffBotSystemHardware::processServoFeedback(const uint8_t* payload, uint8_t length)
{
  if (length < 1)
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                "Empty servo feedback payload");
    return;
  }
  
  uint8_t count = payload[0];
  uint8_t idx = 1;
  
  // Each servo: id(1) + angle(4) = 5 bytes
  for (uint8_t i = 0; i < count; i++)
  {
    if (idx + 5 > length)
    {
      RCLCPP_WARN(rclcpp::get_logger("DiffBotSystemHardware"), 
                  "Insufficient data for servo %d/%d", i+1, count);
      break;
    }
    
    // Parse firmware ID (0-13) and convert to user-facing ID (1-14)
    uint8_t firmware_id = payload[idx++];
    uint8_t servo_id = firmware_id + 1;
    
    // Parse angle (float, little-endian, in degrees)
    float angle_deg;
    memcpy(&angle_deg, &payload[idx], sizeof(float));
    idx += sizeof(float);
    
    // Convert from degrees to radians
    double angle_rad = angle_deg * (M_PI / 180.0);
    
    // Negate angles for elbows and grippers to match simulation direction
    // Servo IDs: 1 (right gripper), 3 (right elbow), 7 (left gripper), 9 (left elbow)
    // Keep positive: 6 (right shoulder), 12 (left shoulder), 13 (neck)
    if (servo_id == 1 || servo_id == 3 || servo_id == 7 || servo_id == 9 || servo_id == 13)
    {
      angle_rad = -angle_rad;
    }
    
    // Store in servo positions map
    servo_positions_[servo_id] = angle_rad;
    servo_feedback_ready_[servo_id] = true;
    
    // Also update hw_positions_ for any joint using this servo
    for (size_t joint_idx : servo_joint_indices_)
    {
      const std::string& joint_name = info_.joints[joint_idx].name;
      auto it = joint_to_servo_id_.find(joint_name);
      if (it != joint_to_servo_id_.end() && it->second == servo_id)
      {
        hw_positions_[joint_idx] = angle_rad;
        break;
      }
    }
  }
}

}  // namespace my_bot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_bot::DiffBotSystemHardware, hardware_interface::SystemInterface)
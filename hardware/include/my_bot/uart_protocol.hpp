#ifndef MY_BOT__UART_PROTOCOL_HPP_
#define MY_BOT__UART_PROTOCOL_HPP_

#include <cstdint>

namespace my_bot
{
namespace uart_protocol
{

// Protocol Constants
constexpr uint8_t HEADER_BYTE1 = 0xAA;
constexpr uint8_t HEADER_BYTE2 = 0x55;
constexpr uint8_t MAX_PAYLOAD_SIZE = 128;

// Command Packet Types (ROS2 -> ESP32)
constexpr uint8_t CMD_PWM                = 0x01;
constexpr uint8_t CMD_ENCODER_REQ        = 0x02;
constexpr uint8_t CMD_FEEDBACK_REQ       = 0x03;
constexpr uint8_t CMD_RESET_ENC          = 0x04;
constexpr uint8_t CMD_CALIBRATE          = 0x05;
constexpr uint8_t CMD_DIAGNOSTICS        = 0x06;
constexpr uint8_t CMD_STOP               = 0x07;
constexpr uint8_t CMD_SERVO_CONTROL      = 0x08;
constexpr uint8_t CMD_SERVO_FEEDBACK_REQ = 0x09;
constexpr uint8_t CMD_SHUTDOWN           = 0x0A;

// Response Packet Types (ESP32 -> ROS2)
constexpr uint8_t RESP_ENCODER        = 0x10;
constexpr uint8_t RESP_FEEDBACK       = 0x11;
constexpr uint8_t RESP_ACK            = 0x12;
constexpr uint8_t RESP_ERROR          = 0x13;
constexpr uint8_t RESP_SERVO_FEEDBACK = 0x14;

// Packet Structure:
// [HEADER1] [HEADER2] [TYPE] [LENGTH] [PAYLOAD...] [CHECKSUM]
//   0xAA      0x55     1byte   1byte    N bytes      1byte

/**
 * @brief Calculate checksum for packet
 * @param packet_type Packet type byte
 * @param length Payload length
 * @param payload Pointer to payload data (can be nullptr if length is 0)
 * @return Calculated checksum byte
 */
inline uint8_t calculate_checksum(uint8_t packet_type, uint8_t length, const uint8_t* payload)
{
  uint8_t checksum = packet_type + length;
  for (uint8_t i = 0; i < length; i++)
  {
    checksum += payload[i];
  }
  return checksum & 0xFF;
}

/**
 * @brief Receive state machine states
 */
enum class RxState : uint8_t
{
  WAIT_HEADER1,
  WAIT_HEADER2,
  WAIT_TYPE,
  WAIT_LENGTH,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM
};

}  // namespace uart_protocol
}  // namespace my_bot

#endif  // MY_BOT__UART_PROTOCOL_HPP_
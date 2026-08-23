// BonicBot A2 — ESP32-S3 USB CDC binary protocol.
//
// Implements bonicbot_usb_cdc_protocol_spec.md framing:
//
//     0xAA | 0x55 | PacketType (1B) | Payload Length (2B LE) | Payload
//
// Note vs. the OLD A2 UART protocol this replaces: the length field grew from
// 1 to 2 bytes and the trailing checksum byte is GONE. Framing is identical to
// the BLE channel, so bonicbot_ble_protocol_spec.md's payload tables apply here
// verbatim — that document is authoritative where it and the CDC spec disagree
// (the CDC spec's CMD_SERVO_CONTROL table predates the unified firmware).
//
// Unlike M1 — where the Jetson drives ODrive/QDD/servos directly over CAN and
// the ESP only relays IMU/Wi-Fi — on A2 the ESP32 IS the motion controller.
// This host therefore drives the full motion command set, not a relay subset.

#ifndef BONICBOT_A2_HARDWARE__CDC_PROTOCOL_HPP_
#define BONICBOT_A2_HARDWARE__CDC_PROTOCOL_HPP_

#include <cstdint>

namespace bonicbot_a2_hardware
{
namespace cdc_protocol
{

// ── Framing ────────────────────────────────────────────────────────────
constexpr uint8_t  MAGIC1 = 0xAA;
constexpr uint8_t  MAGIC2 = 0x55;
constexpr uint16_t MAX_PAYLOAD_SIZE = 256;   // firmware drops oversized frames
constexpr uint8_t  HEADER_SIZE = 5;          // magic1 + magic2 + type + len(2B)

// ── Commands (host -> ESP) ─────────────────────────────────────────────
constexpr uint8_t CMD_PING                  = 0x01;  // liveness; replies RESP_ACK
constexpr uint8_t CMD_MOTOR_MOVE            = 0x02;  // wheel velocities (m/s)
constexpr uint8_t CMD_SERVO_MULTI           = 0x0A;  // servo POSITIONS (registry IDs)
constexpr uint8_t CMD_ENCODER_REQUEST       = 0x21;
constexpr uint8_t CMD_RESET_ENCODERS        = 0x23;
constexpr uint8_t CMD_CALIBRATE             = 0x24;
constexpr uint8_t CMD_DIAGNOSTICS           = 0x25;
constexpr uint8_t CMD_STOP                  = 0x26;  // emergency stop both motors
constexpr uint8_t CMD_SERVO_CONTROL         = 0x27;  // single-servo ACTION, 2B payload
constexpr uint8_t CMD_IMU_REQUEST           = 0x29;
constexpr uint8_t CMD_SERVO_FEEDBACK_REQUEST = 0x2A;

// Wi-Fi relay (CDC spec §7). 0x0B arrives FROM the ESP (phone wrote it over
// BLE); 0x0C is bidirectional — ESP asks, host replies with the status payload.
constexpr uint8_t CMD_WIFI_CONFIG = 0x0B;
constexpr uint8_t CMD_WIFI_STATUS = 0x0C;

// ── Responses (ESP -> host) ────────────────────────────────────────────
constexpr uint8_t RESP_ACK             = 0x50;
constexpr uint8_t RESP_NACK            = 0x51;
constexpr uint8_t RESP_BATTERY         = 0x52;  // 31B: V, A, SOC%, servo census
constexpr uint8_t RESP_ENCODERS        = 0x60;  // 8B: two int32 tick counts
constexpr uint8_t RESP_SERVO_FEEDBACK  = 0x61;  // 1 + N*5: count + [id, float deg]
constexpr uint8_t RESP_IMU             = 0x63;  // 24B: 6 floats

// ── Payload sizes ──────────────────────────────────────────────────────
constexpr uint16_t IMU_PAYLOAD_SIZE      = 24;  // ax ay az (m/s^2), gx gy gz (deg/s)
constexpr uint16_t ENCODER_PAYLOAD_SIZE  = 8;   // int32 left, int32 right
constexpr uint16_t BATTERY_PAYLOAD_SIZE  = 31;  // 3 floats + count + uint8[18]
constexpr uint16_t WIFI_STATUS_PAYLOAD_SIZE = 50;  // u8 + char[32] + u8 + char[16]
constexpr uint8_t  SERVO_MULTI_ENTRY_SIZE = 8;  // id + float angle + u16 speed + u8 accel
constexpr uint8_t  SERVO_FEEDBACK_ENTRY_SIZE = 5;  // id + float angle

// ── CMD_SERVO_CONTROL (0x27) action codes ──────────────────────────────
// Payload is exactly 2 bytes: [Servo ID][Action]. This command CANNOT carry an
// angle — positional moves go out on CMD_SERVO_MULTI (0x0A).
enum class ServoAction : uint8_t
{
  MOVE_TO_TARGET = 0,
  SET_MIDDLE     = 1,   // calibrate zero offset
  TORQUE_OFF     = 2,
  TORQUE_ON      = 3,
};

// ── CMD_SERVO_FEEDBACK_REQUEST (0x2A) modes ────────────────────────────
// Payload: [Mode][uint16 interval_ms LE][Count][IDs...]. Continuous mode makes
// the ESP stream RESP_SERVO_FEEDBACK on its own, replacing per-cycle polling.
enum class FeedbackMode : uint8_t
{
  ONCE       = 0,
  CONTINUOUS = 1,
  STOP       = 2,
};
constexpr uint16_t FEEDBACK_MIN_INTERVAL_MS = 50;

// ── Receive state machine ──────────────────────────────────────────────
// One state shorter than the old protocol's — there is no checksum to wait for.
enum class RxState : uint8_t
{
  WAIT_MAGIC1,
  WAIT_MAGIC2,
  WAIT_TYPE,
  WAIT_LENGTH_LOW,
  WAIT_LENGTH_HIGH,
  WAIT_PAYLOAD,
};

}  // namespace cdc_protocol
}  // namespace bonicbot_a2_hardware

#endif  // BONICBOT_A2_HARDWARE__CDC_PROTOCOL_HPP_

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

namespace esphome {
namespace wintex {

enum class WintexDatapointType : uint8_t {
  ZONE_STATUS = 0x00,      // bitmask, 1 byte
  // BOOLEAN = 0x01,  // 1 byte (0/1)
  // INTEGER = 0x02,  // 4 byte
  // STRING = 0x03,   // variable length
  // ENUM = 0x04,     // 1 byte
  // BITMASK = 0x05,  // 2 bytes
};

struct WintexDatapoint {
  uint8_t id;
  WintexDatapointType type;
  size_t len;
  union {
    bool value_bool;
    uint32_t value_uint;
    uint8_t value_enum;
    uint32_t value_bitmask;
  };
  std::string value_string;
  std::vector<uint8_t> value_raw;
};

struct WintexDatapointListener {
  uint8_t datapoint_id;
  std::function<void(WintexDatapoint)> on_datapoint;
};

enum class WintexCommandType : uint8_t {
  SESSION = 'Z',
  READ_CONFIGURATION = 'O',
  READ_VOLATILE = 'R',
  WRITE_VOLATILE = 'W',
  HANGUP = 'H',
};

enum class WintexResponseType : uint8_t {
  SESSION = 'Z',
  READ_CONFIGURATION = 'I',
  READ_VOLATILE = 'W',
  HANGUP = 0x0F,
};

enum class WintexInitState : uint8_t {
  UNAUTH = 0x00,
  AUTH,
  INIT,
  INIT_DONE,
  MESSAGE_SENT,
};

struct WintexCommand {
  WintexCommandType cmd;
  std::vector<uint8_t> payload;
};

class Wintex : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void register_listener(uint8_t datapoint_id, const std::function<void(WintexDatapoint)> &func);
  void set_udl(std::string udl) { this->udl_ = udl; }

 protected:
  void handle_char_(uint8_t c);
  void handle_datapoint_(const uint8_t *buffer, size_t len);
  void handle_zone_(uint8_t zone, uint8_t status);
  optional<WintexDatapoint> get_datapoint_(uint8_t datapoint_id);
  bool validate_message_();

  void handle_command_(WintexResponseType command, const uint8_t *buffer, size_t len);
  void send_raw_command_(WintexCommand command);
  void process_command_queue_();
  void send_command_(WintexCommand command);
  void send_empty_command_(WintexCommandType command);
//  void send_datapoint_command_(uint8_t datapoint_id, WintexDatapointType datapoint_type, std::vector<uint8_t> data);

  WintexInitState init_state_ = WintexInitState::UNAUTH;
  uint32_t last_command_timestamp_ = 0;
  std::string product_ = "";
  std::string udl_ = "";
  std::vector<WintexDatapointListener> listeners_;
  std::vector<WintexDatapoint> datapoints_;
  std::vector<uint8_t> rx_message_;
  std::vector<WintexCommand> command_queue_;
};

}  // namespace wintex
}  // namespace esphome

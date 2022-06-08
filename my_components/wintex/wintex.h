#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

namespace esphome {
namespace wintex {

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

class WintexSensorBase {
  public:
    virtual ~WintexSensorBase() {}
    virtual void update_wintex(const uint8_t *memory);
    uint32_t get_address() { return this->address_; }
    uint8_t get_length() { return this->length_; }
    uint8_t get_offset() { return this->offset_; }

  protected:
    void set_wintex_address(uint32_t address, uint8_t length, uint8_t offset) {
      this->address_ = address;
      this->length_ = length;
      this->offset_ = offset;
    }
  uint32_t address_{0};
  uint8_t length_{0};
  uint8_t offset_{0};
};

class Wintex : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_udl(std::string udl) { this->udl_ = udl; }
  void register_sensor(WintexSensorBase *sensor);

 protected:
  void send_command_(WintexCommand command);

 private:
  void handle_char_(uint8_t c);
  bool validate_message_();

  void handle_command_(WintexResponseType command, const uint8_t *buffer, size_t len);
  void send_raw_command_(WintexCommand command);
  void process_command_queue_();
  void send_empty_command_(WintexCommandType command);
  WintexCommand volatile_read(uint32_t address, uint8_t length);

  void update_sensors_(uint32_t address, uint8_t length, const uint8_t *data);

  WintexInitState init_state_ = WintexInitState::UNAUTH;
  uint32_t last_command_timestamp_ = 0;
  std::string product_ = "";
  std::string udl_ = "";
  std::vector<WintexSensorBase*> sensors_;
  std::vector<uint8_t> rx_message_;
  std::vector<WintexCommand> command_queue_;
  uint16_t current_sensor_{0};
};

}  // namespace wintex
}  // namespace esphome

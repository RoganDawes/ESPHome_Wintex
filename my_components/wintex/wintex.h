#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/application.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"

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
  LOGIN_FAILED,
  AUTH,
  ZONE_QUERY,
  INIT_DONE,
  IDLE,
  MESSAGE_SENT,
};

struct WintexCommand {
  WintexCommandType cmd;
  std::vector<uint8_t> payload;
};

enum class WintexBinarySensorType {
  ZONE_STATUS,
  ZONE_TAMPER,
  ZONE_TEST,
  ZONE_ALARMED,
  ZONE_BYPASSED,
  ZONE_AUTO_BYPASSED,
  ZONE_FAULTY,
  PARTITION_n_BELL,
  PARTITION_n_STROBE,
  PARTITION_n_PA_ALARM,
  PARTITION_n_DURESS_ALARM,
  PARTITION_n_BURGLAR_ALARM,
  PARTITION_n_ARMED,
  SYSTEM_LATCHED_AC_FAIL,
  PARTITION_n_ARMED_ALARM,
  PANEL_BOX_TAMPER_OPEN,
  AUX_INPUT_ACTIVE,
  LOCAL_EXPANDER_OFFLINE,
  AUX_12V_FUSE_BLOWN,
  PARTITION_FIRE_ALARM,
  PARTITION_PA_ALARM,
  PARTITION_BURGLAR_ALARM,
  PARTITION_ARMED,
  COURTESY_LIGHT,
  SUCCESSFUL_TRANSMISSION,
  LOG_80PCT_FULL,
  PROGRAM_MODE_SELECTED,
  DOWNLOAD_IN_PROGRESS,
  TIMED_ARMING_COUNTDOWN,
  ZONE_SOAK_TEST_ACTIVE,
  ZONE_SOAK_TEST_FAILED,
  CONTROL_TIMER,
  PC_CONTROL,

};

class Wintex;

class WintexSensorBase {
  friend class Wintex;
  public:
    WintexSensorBase(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset) {
      wintex_ = wintex;
      address_ = address;
      length_ = length;
      offset_ = offset;
    }
    uint32_t get_address() { return this->address_; }
    uint8_t get_length() { return this->length_; }
    uint8_t get_offset() { return this->offset_; }

  protected:
    virtual void update_state(const uint8_t *memory);
  Wintex *wintex_;
  uint32_t address_{0};
  uint8_t length_{0};
  uint8_t offset_{0};
};

class WintexBinarySensor : public WintexSensorBase, public binary_sensor::BinarySensor {
 public:
  WintexBinarySensor(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset, uint8_t mask) 
    : WintexSensorBase(wintex, address, length, offset) {
    mask_ = mask;
  }
  void setup() {};
  void dump_config() {};

 protected:
  void update_state(const uint8_t *memory) {
    this->publish_state(memory[offset_] & mask_);
  };
  uint8_t mask_;
};

#ifdef FOOBAR
class WintexSensor : public WintexSensorBase, public sensor::Sensor {
 public:
  WintexSensor(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset) 
    : WintexSensorBase(wintex, address, length, offset) {
  }
  void setup() {};
  void dump_config() {};

 protected:
  void update_state(const uint8_t *memory) {
    this->publish_state(memory[offset_] / 255 * 17.93);
  };
};

class WintexSwitch : public WintexSensorBase, public switch_::Switch {
  public:
    WintexSwitch(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset, uint8_t mask, WintexCommand on_command, WintexCommand off_command) 
      : WintexSensorBase(wintex, address, length, offset) {
        this->mask_ = mask;
        this->on_command_ = on_command;
        this->off_command_ = off_command;
    }
    void setup() {};
    void dump_config() {};
    void turn_on();
    void turn_off();
    void toggle() {
      if (this->state) {
        this->turn_off();
      } else {
        this->turn_on();
      }
    };

    /**
     * @brief creates a command that can be passed to set_switch_commands to bypass a particular
     * zone.
     * 
     * @param base_address The address of the start of the zone state range.
     * @param zone a 1-based zone number
     * @param state bypass on (true) or off (false)
     * @return WintexCommand 
     */
    static WintexCommand bypassZoneCommand(uint32_t base_address, uint8_t zone, bool state) {
      uint32_t address = base_address + zone - 1;
      uint8_t a0 = (address >> 16) & 0xff;
      uint8_t a1 = (address >> 8) & 0xff;
      uint8_t a2 = (address) & 0xff;
      return WintexCommand{.cmd = WintexCommandType::WRITE_VOLATILE, .payload = std::vector<uint8_t>{a0, a1, a2, 0x01, (state ? (char) 0xa0 : (char) 0x00)}};
    }

  protected:
    uint8_t mask_;
    WintexCommand on_command_, off_command_;
    void update_state(const uint8_t *memory) {
      this->publish_state(memory[offset_] & mask_);
    };
};
#endif

/**
 * @brief WintexZone represents an entire zone, with a number of binary_sensors
 * We expose the zone's status directly as a binary_sensor using a callback
 * that mirrors the zone_status sensor to the zone itself.
 * 
 */
class WintexZone : public binary_sensor::BinarySensor {
  public:
  WintexZone(uint16_t zone) {
    zone_ = zone;
  }
  // called after the hub has queried the various base addresses,
  // as well as the zone names, from the panel
  // Can be overridden by YAML configuration.
  void setup(Wintex *wintex, uint32_t zone_base_address, uint16_t zone_group_size, std::string zone_name);
  WintexBinarySensor *status, *tamper, *test, *alarmed, *bypassed, *auto_bypassed;

  private:
    uint16_t zone_;
};

class Wintex : public Component, public uart::UARTDevice {
  friend class WintexZone;
  friend class WintexSwitch;
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_udl(std::string udl) { this->udl_ = udl; }
  void register_zone(WintexZone *zone);

 protected:
  void send_command_(WintexCommand command);
  void register_sensor(WintexSensorBase *sensor);

 private:
  void handle_char_(uint8_t c);
  bool validate_message_();
  void setup_zones_();

  void handle_command_(WintexResponseType command, const uint8_t *buffer, size_t len);
  void send_raw_command_(WintexCommand command);
  void process_command_queue_();
  void send_empty_command_(WintexCommandType command);
  WintexCommand volatile_read(uint32_t address, uint8_t length);

  void update_sensors_(uint32_t address, uint8_t length, const uint8_t *data);

  WintexInitState init_state_ = WintexInitState::UNAUTH;
  uint32_t last_command_timestamp_{0};
  std::string product_ = "";
  std::string udl_ = "";
  std::vector<WintexSensorBase*> sensors_;
  std::vector<WintexZone *> zones_;
  std::vector<uint8_t> rx_message_;
  std::vector<WintexCommand> command_queue_;
  uint16_t current_sensor_{0};
};

}  // namespace wintex
}  // namespace esphome

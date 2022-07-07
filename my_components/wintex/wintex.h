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
  COMMIT = 'U',
  HANGUP = 'H',
  READ_CONFIGURATION = 'O',
  READ_VOLATILE = 'R',
  SESSION = 'Z',
  WRITE_VOLATILE = 'W',
};

enum class WintexResponseType : uint8_t {
  SESSION = 'Z',
  READ_CONFIGURATION = 'I',
  READ_VOLATILE = 'W',
  ACK = 0x06,
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

struct WintexResponse {
  WintexResponseType type;
  size_t len;
  const uint8_t *data;
};

class WintexResponseHandler {
  public:
    void handle_response(WintexResponse response);
};

struct WintexCommand {
  WintexCommandType cmd;
  std::vector<uint8_t> payload;
};

struct AsyncWintexCommand {
  WintexCommandType cmd;
  std::vector<uint8_t> payload;
  std::function<optional<AsyncWintexCommand>(WintexResponse)> callback;
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
    WintexSensorBase(uint32_t address, uint8_t length, uint8_t offset) {
      address_ = address;
      length_ = length;
      offset_ = offset;
    }
    uint32_t get_address() { return this->address_; }
    uint8_t get_length() { return this->length_; }

  protected:
    virtual void update_state(const uint8_t *memory) = 0;
    uint32_t address_{0};
    uint8_t length_{0};
    uint8_t offset_{0};
};

class WintexBinarySensor : public WintexSensorBase, public binary_sensor::BinarySensor {
 public:
  WintexBinarySensor(uint32_t address, uint8_t length, uint8_t offset, uint8_t mask) 
    : WintexSensorBase(address, length, offset) {
    mask_ = mask;
  }

 protected:
  void update_state(const uint8_t *memory) override {
    this->publish_state(memory[offset_] & mask_);
  };
  uint8_t mask_;
};

class WintexSensor : public WintexSensorBase, public sensor::Sensor {
 public:
  WintexSensor(uint32_t address, uint8_t length, uint8_t offset) 
    : WintexSensorBase(address, length, offset) {
  }

 protected:
  void update_state(const uint8_t *memory) {
    this->publish_state(memory[offset_] / 255 * 17.93);
  };
};

class WintexSwitch : public WintexSensorBase, public WintexResponseHandler, public switch_::Switch {
  public:
    WintexSwitch(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset, uint8_t mask) 
      : WintexSensorBase(address, length, offset) {
        this->wintex_ = wintex;
        this->mask_ = mask;
    }
    virtual void write_state(bool state) = 0;

  protected:
    Wintex *wintex_;
    uint8_t mask_;
    void update_state(const uint8_t *memory) override {
      this->publish_state(memory[offset_] & mask_);
    };
    virtual optional<AsyncWintexCommand> handle_response(WintexResponse response) = 0;
    std::function<optional<AsyncWintexCommand>(WintexResponse)> callback_ = [this](WintexResponse response) {
      return this->handle_response(response);
    };
};

class WintexZoneBypassSwitch: public WintexSwitch {
  friend class Wintex;
  public:
    WintexZoneBypassSwitch(Wintex *wintex, uint32_t address, uint8_t length, uint8_t offset) 
    : WintexSwitch (wintex, address, length, offset, 0x20) {}
    void write_state(bool state) override;

  protected:
    optional<AsyncWintexCommand> handle_response(WintexResponse response) override;
  private:
    bool commit_{false};
};

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
  WintexBinarySensor *status, *tamper, *test, *alarmed,*auto_bypassed;
  WintexSwitch *bypass;

  private:
    uint16_t zone_;
    bool setup_{false};
};

class Wintex : protected WintexResponseHandler, public Component, public uart::UARTDevice {
  friend class WintexZone;
  friend class WintexZoneBypassSwitch;
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_udl(std::string udl) { this->udl_ = udl; }
  void register_zone(WintexZone *zone);

 protected:
  void queue_command_(AsyncWintexCommand command);
  void register_sensor(WintexSensorBase *sensor);

 private:
  void handle_char_(uint8_t c);
  bool command_queue_is_locked_();
  void send_command_now_(AsyncWintexCommand command);
  optional<WintexResponse> parse_response_();
  void setup_zones_();
  void update_sensors_();

  optional<AsyncWintexCommand> handle_login_(WintexResponse response);
  optional<AsyncWintexCommand> handle_heartbeat_(WintexResponse response);
  optional<AsyncWintexCommand> handle_sensors_(WintexResponse response);

  void process_command_queue_();

  WintexInitState init_state_ = WintexInitState::UNAUTH;
  uint32_t last_command_timestamp_{0};
  std::string product_ = "";
  std::string udl_ = "";
  AsyncWintexCommand login_;
  std::vector<WintexSensorBase*> sensors_;
  std::vector<WintexZone *> zones_;
  std::vector<uint8_t> rx_message_;
  std::vector<AsyncWintexCommand> command_queue_;
  uint16_t current_sensor_{0};
  optional<AsyncWintexCommand> current_command_;
  optional<WintexResponse> sensor_response_;
};

}  // namespace wintex
}  // namespace esphome

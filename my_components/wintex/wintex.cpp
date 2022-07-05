#include "wintex.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace wintex {

static const char *TAG = "wintex";
static const int COMMAND_DELAY = 1000;

/*
void WintexSwitch::turn_on() {
  this->wintex_->send_command_(this->on_command_);
};
void WintexSwitch::turn_off() {
  this->wintex_->send_command_(this->off_command_);
};
*/

void WintexZone::setup(Wintex *wintex, uint32_t zone_base_address, uint16_t zone_group_size, std::string zone_name) {
  if (get_name() == "")
    set_name(zone_name);
  std::string name = get_name();
  uint16_t zone = this->zone_ - 1;
  status = new WintexBinarySensor(wintex, zone_base_address, zone_group_size, zone, 0x01);
  status->set_name(name + " status");
  status->add_on_state_callback([this](bool state) {
    if (!state)
      this->publish_state(state);
  });
  tamper = new WintexBinarySensor(wintex, zone_base_address, zone_group_size, zone, 0x02);
  tamper->set_name(name + " tamper");
  test = new WintexBinarySensor(wintex, zone_base_address, zone_group_size, zone, 0x08);
  test->set_name(name + " test");
  alarmed = new WintexBinarySensor(wintex, zone_base_address, zone_group_size, zone, 0x10);
  alarmed->set_name(name + " alarmed");
  bypassed = new WintexBinarySensor(wintex, zone_base_address, zone_group_size, zone, 0x20);
  bypassed->set_name(name + " bypassed");
  auto_bypassed = new WintexBinarySensor(wintex, zone_base_address, zone_group_size, zone, 0x40);
  auto_bypassed->set_name(name + " auto bypassed");
  // Should only enable this once we are sorting the sensors by base address
  // faulty = new WintexBinarySensor(wintex, zone_base_address + 0x20, zone_group_size, zone_, 0x02);
  // faulty->set_name(name + " faulty");
  wintex->register_sensor(status);
  App.register_binary_sensor(status);
  wintex->register_sensor(tamper);
  App.register_binary_sensor(tamper);
  wintex->register_sensor(test);
  App.register_binary_sensor(test);
  wintex->register_sensor(alarmed);
  App.register_binary_sensor(alarmed);
  wintex->register_sensor(bypassed);
  App.register_binary_sensor(bypassed);
  wintex->register_sensor(auto_bypassed);
  App.register_binary_sensor(auto_bypassed);
  // wintex->register_sensor(faulty);
  // App.register_binary_sensor(faulty);
}

void Wintex::setup() {
  last_command_timestamp_ = millis();
}

void Wintex::loop() {
  while (available()) {
    uint8_t c;
    read_byte(&c);
    handle_char_(c);
  }
  if (init_state_ == WintexInitState::UNAUTH && millis() - last_command_timestamp_ > 10000) {
    send_command_(WintexCommand{.cmd = WintexCommandType::SESSION, .payload = std::vector<uint8_t>{udl_.begin(), udl_.end()}});
  }
  process_command_queue_();
}

void Wintex::dump_config() {
  ESP_LOGCONFIG(TAG, "Wintex:");
  if (init_state_ != WintexInitState::INIT_DONE) {
    ESP_LOGCONFIG(TAG, "  Configuration will be reported when setup is complete. Current init_state: %u",
                  static_cast<uint8_t>(init_state_));
    ESP_LOGCONFIG(TAG, "  If no further output is received, confirm that this is a supported Wintex device.");
    return;
  }
  ESP_LOGCONFIG(TAG, "  Product: '%s'", product_.c_str());
}

bool Wintex::validate_message_() {
  uint32_t at = rx_message_.size() - 1;
  auto *data = &rx_message_[0];
  
  size_t length = data[0];
  // Byte 0: Length - unvalidated
  if (at == 0)
    return true;

  // Byte 1: command
  WintexResponseType command = (WintexResponseType) data[1];
  if (at == 1)
    switch(command) {
      case WintexResponseType::SESSION:
      case WintexResponseType::HANGUP:
      case WintexResponseType::READ_CONFIGURATION:
      case WintexResponseType::READ_VOLATILE:
        return true;
      default: return false;
    }

  // no validation for the data fields - if any
  // wait until all data is read
  if (at < length - 1)
    return true;

  // validate checksum
  // Byte LEN: CHECKSUM - sum of all bytes (including header) ^ 0xFF
  uint8_t rx_checksum = data[at];
  uint8_t calc_checksum = 0;
  for (uint8_t i = 0; i < length - 1; i++)
    calc_checksum += data[i];
  calc_checksum ^= 0xFF;

  if (rx_checksum != calc_checksum) {
    ESP_LOGW(TAG, "Wintex Received invalid message checksum CMD=0x%02x DATA=[%s] INIT_STATE=%u Checksum: %02X!=%02X", static_cast<uint8_t>(command),
           format_hex_pretty(data + 2, length - 3).c_str(), static_cast<uint8_t>(init_state_), rx_checksum, calc_checksum);
    return false;
  }

  // valid message
  const uint8_t *message_data = data + 2;
  ESP_LOGV(TAG, "Received Wintex: CMD=0x%02X DATA=[%s] INIT_STATE=%u", static_cast<uint8_t>(command),
           format_hex_pretty(message_data, length - 3).c_str(), static_cast<uint8_t>(init_state_));
  handle_command_(command, message_data, length - 3);

  // return false to reset rx buffer
  return false;
}

void Wintex::handle_char_(uint8_t c) {
  rx_message_.push_back(c);
  if (!validate_message_()) {
    rx_message_.clear();
  }
}

void Wintex::handle_command_(WintexResponseType command, const uint8_t *buffer, size_t len) {
  switch (command) {
    case WintexResponseType::HANGUP:
      ESP_LOGV(TAG, "Logged out");
      init_state_ = WintexInitState::UNAUTH;
      send_command_(WintexCommand{.cmd = WintexCommandType::SESSION, .payload = std::vector<uint8_t>{0x01, 0x09, 0x06, 0x07}});
      break;
    case WintexResponseType::SESSION:
      if (len == 16) {
        // check it is a valid string made up of printable characters
        bool valid = true;
        for (int i = 0; i < len; i++) {
          if (!std::isprint(buffer[i])) {
            valid = false;
            break;
          }
        }
        if (valid) {
          product_ = std::string(reinterpret_cast<const char *>(buffer), len);
          ESP_LOGV("Successful authentication, product [%s]", product_.c_str());
          init_state_ = WintexInitState::AUTH;
          setup_zones_();
          update_sensors_(0, 0, nullptr);
        } else {
          product_ = R"({"p":"INVALID"})";
        }
      } else if (len == 0x09) {
        ESP_LOGV("Heartbeat response [%s]", format_hex_pretty(buffer, len).c_str());
      }
      break;
    case WintexResponseType::READ_CONFIGURATION:
      // FIXME: not yet implemented
      break;
    case WintexResponseType::READ_VOLATILE: {
      uint32_t address = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
      uint8_t length = buffer[3];
      const uint8_t *data = &buffer[4];
      update_sensors_(address, length, data);
      break;
    }
    default:
      ESP_LOGE(TAG, "Invalid command (0x%02X) received", static_cast<uint8_t>(command));
  }
}

void Wintex::update_sensors_(uint32_t address, uint8_t length, const uint8_t *data) {
  if (sensors_.size() == 0) {
    ESP_LOGV(TAG, "No sensors to update");
    return;
  }
  while (current_sensor_ < sensors_.size()) {
    auto sensor = sensors_[current_sensor_];
    if (address == sensor->get_address() && length == sensor->get_length()) {
      sensor->update_state(data);
      current_sensor_++;
    } else {
      ESP_LOGV(TAG, "Changing requested address at sensor %d", current_sensor_);
      break;
    }
  }
  current_sensor_ = current_sensor_ % sensors_.size();
  auto sensor = sensors_[current_sensor_];
  send_command_(volatile_read(sensor->get_address(), sensor->get_length()));
}

WintexCommand Wintex::volatile_read(uint32_t address, uint8_t length) {
  uint8_t a0 = (address >> 16) & 0xff;
  uint8_t a1 = (address >> 8) & 0xff;
  uint8_t a2 = (address) & 0xff;
  return WintexCommand{.cmd = WintexCommandType::READ_VOLATILE, .payload = std::vector<uint8_t>{a0, a1, a2, length}}; 
}

void Wintex::send_raw_command_(WintexCommand command) {
  last_command_timestamp_ = millis();
  uint8_t len = (uint8_t)(command.payload.size()) + 3;

  ESP_LOGV(TAG, "Sending Wintex: CMD=0x%02X DATA=[%s] INIT_STATE=%u", static_cast<uint8_t>(command.cmd),
           format_hex_pretty(command.payload).c_str(), static_cast<uint8_t>(init_state_));

  write_array({len, (uint8_t) command.cmd});
  if (!command.payload.empty())
    write_array(command.payload.data(), command.payload.size());

  uint8_t checksum = len + (uint8_t) (command.cmd);
  for (auto &data : command.payload)
    checksum += data;
  checksum ^= 0xFF;

  write_byte(checksum);
}

void Wintex::process_command_queue_() {
  uint32_t delay = millis() - last_command_timestamp_;
  // Left check of delay since last command in case theres ever a command sent by calling send_raw_command_ directly
  if (delay > COMMAND_DELAY && !command_queue_.empty()) {
    send_raw_command_(command_queue_.front());
    command_queue_.erase(command_queue_.begin());
  }
}

void Wintex::register_sensor(WintexSensorBase *sensor){
  sensors_.push_back(sensor);
}

void Wintex::register_zone(WintexZone *zone){
  zones_.push_back(zone);
}

void Wintex::setup_zones_(){
  for (WintexZone *zone: this->zones_) {
    zone->setup(this, (uint32_t) 0x4f8, (uint16_t) 0x20, "");
  }
}

void Wintex::send_command_(WintexCommand command) {
  command_queue_.push_back(command);
  process_command_queue_();
}

void Wintex::send_empty_command_(WintexCommandType command) {
  send_command_(WintexCommand{.cmd = command, .payload = std::vector<uint8_t>{}});
}

}  // namespace wintex
}  // namespace esphome

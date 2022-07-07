#include "wintex.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace wintex {

static const char *TAG = "wintex";
static const int COMMAND_DELAY = 1000;

static std::vector<uint8_t> read_payload(uint32_t address, uint8_t length) {
  auto payload = std::vector<uint8_t>();
  payload.push_back((address >> 16) & 0xff);
  payload.push_back((address >> 8) & 0xff);
  payload.push_back((address) & 0xff);
  payload.push_back(length);
  return payload;
};
static std::vector<uint8_t> write_payload(uint32_t address, std::vector<uint8_t> data) {
  assert(data.size() <= 0xff);
  auto payload = read_payload(address, data.size() & 0xff);
  for (auto it = data.cbegin(); it != data.cend(); ++it) {
    payload.push_back(*it);
  }
  return payload;
};

void WintexZoneBypassSwitch::write_state(bool state) {
  uint32_t address = address_ + offset_;
  uint8_t data = (state ? 0xa0 : 0x80);
  auto payload = write_payload(address, {data});
  auto bypass = AsyncWintexCommand{
    .cmd = WintexCommandType::WRITE_VOLATILE, 
    .payload = payload, 
    .callback = callback_
  };
  this->commit_ = true;
  this->wintex_->queue_command_(bypass);
}

optional<AsyncWintexCommand> WintexZoneBypassSwitch::handle_response(WintexResponse response) {
  if (response.type != WintexResponseType::ACK) {
    ESP_LOGE(TAG, "Unexpected response to command: %d", (uint8_t) response.type);
  } else {
    if (commit_) {
      auto commit = AsyncWintexCommand{
        .cmd = WintexCommandType::COMMIT,
        .payload = {},
        .callback = this->callback_,
      };
      this->commit_= false;
      return commit;
    } else {
      // Acknowledge new state by publishing it
      publish_state(!state);
    }
  }
  return {};
}

void WintexZone::setup(Wintex *wintex, uint32_t zone_base_address, uint16_t zone_group_size, std::string zone_name) {
  if (setup_)
    return;
  setup_ = true;
  if (get_name() == "")
    set_name(zone_name);
  std::string name = get_name();
  App.register_binary_sensor(this);
  uint16_t zone = this->zone_ - 1;
  status = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x01);
  status->set_name(name + " status");
  status->set_internal(true);
  status->add_on_state_callback([this](bool state) {
    this->publish_state(state);
  });
  wintex->register_sensor(status);
  App.register_binary_sensor(status);
  tamper = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x02);
  tamper->set_name(name + " tamper");
  wintex->register_sensor(tamper);
  App.register_binary_sensor(tamper);
  test = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x08);
  test->set_name(name + " test");
  wintex->register_sensor(test);
  App.register_binary_sensor(test);
  alarmed = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x10);
  alarmed->set_name(name + " alarmed");
  wintex->register_sensor(alarmed);
  App.register_binary_sensor(alarmed);
  bypass = new WintexZoneBypassSwitch(wintex, zone_base_address, zone_group_size, zone);
  bypass->set_name(name + " bypassed");
  wintex->register_sensor(bypass);
  App.register_switch(bypass);
  auto_bypassed = new WintexBinarySensor(zone_base_address, zone_group_size, zone, 0x40);
  auto_bypassed->set_name(name + " auto bypassed");
  wintex->register_sensor(auto_bypassed);
  App.register_binary_sensor(auto_bypassed);
  // Should only enable this once we are sorting the sensors by base address
  // faulty = new WintexBinarySensor(zone_base_address + 0x20, zone_group_size, zone_, 0x02);
  // faulty->set_name(name + " faulty");
  // wintex->register_sensor(faulty);
  // App.register_binary_sensor(faulty);
}

void Wintex::setup() {
  last_command_timestamp_ = millis();
  this->login_ = AsyncWintexCommand{
    .cmd = WintexCommandType::SESSION,
    .payload = std::vector<uint8_t>{udl_.begin(), udl_.end()},
    .callback = [this](WintexResponse response) {
        return this->handle_login_(response);
      },
    };
  this->queue_command_(this->login_);
}

optional<AsyncWintexCommand> Wintex::handle_login_(WintexResponse response) {
  if (response.type == WintexResponseType::SESSION) {
    if (response.len == 16) {
      // check it is a valid string made up of printable characters
      bool valid = true;
      for (int i = 0; i < response.len; i++) {
        if (!std::isprint(response.data[i])) {
          valid = false;
          break;
        }
      }
      if (valid) {
        product_ = std::string(reinterpret_cast<const char *>(response.data), response.len);
        ESP_LOGV("Successful authentication, product [%s]", product_.c_str());
        init_state_ = WintexInitState::AUTH;
        setup_zones_();
        this->update_sensors_();
      }
    }
  }
  ESP_LOGE(TAG, "Authentication failed");
  product_ = R"({"p":"INVALID"})";
  return {};
}

optional<AsyncWintexCommand> Wintex::handle_heartbeat_(WintexResponse response) {
  if (response.type == WintexResponseType::SESSION 
    && response.len == 0x09) {
      ESP_LOGV("Heartbeat response [%s]", format_hex_pretty(response.data, response.len).c_str());
  } else {
    ESP_LOGE(TAG, "Unexpected heartbeat response: %d", (uint8_t) response.type);
  }
  return {};
}

void Wintex::loop() {
  while (available()) {
    uint8_t c;
    read_byte(&c);
    handle_char_(c);
  }
  if (millis() - last_command_timestamp_ > 10000) {
    queue_command_(login_);
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

optional<WintexResponse> Wintex::parse_response_() {
  size_t length = rx_message_[0];

  if (rx_message_.size() < length)
    return {};

  ESP_LOGVV(TAG, "Received message DATA=[%s]", format_hex_pretty(&rx_message_[0], length).c_str());

  // validate checksum
  // Byte LEN: CHECKSUM - sum of all bytes (including header) ^ 0xFF
  uint8_t rx_checksum = rx_message_[length-1];
  uint8_t calc_checksum = 0;
  for (uint8_t i = 0; i < length - 1; i++)
    calc_checksum += rx_message_[i];
  calc_checksum ^= 0xFF;

  if (rx_checksum != calc_checksum) {
    ESP_LOGW(TAG, "Wintex Received invalid message checksum DATA=[%s] Checksum: %02X!=%02X", 
      format_hex_pretty(&rx_message_[0], length).c_str(), rx_checksum, calc_checksum);
    this->rx_message_.clear();
    return {};
  }

  // valid message
  WintexResponseType type = (WintexResponseType) rx_message_[1];
  WintexResponse response = WintexResponse{
    .type = type,
    .len = length - 3,
    .data = &rx_message_[2]
  };
  ESP_LOGV(TAG, "Received Response: type=0x%02X DATA=[%s]", static_cast<uint8_t>(response.type),
           format_hex_pretty(response.data, response.len).c_str());
  return response;
}

void Wintex::handle_char_(uint8_t c) {
  rx_message_.push_back(c);
  optional<WintexResponse> response = this->parse_response_();
  if (response.has_value() && this->current_command_.has_value()) {
    std::function<optional<AsyncWintexCommand>(WintexResponse)> callback = this->current_command_->callback;
    this->current_command_ = {};
    optional<AsyncWintexCommand> command = callback(response.value());
    rx_message_.clear();
    if (command.has_value()) {
      this->send_command_now_(command.value());
    }
  }
}

void Wintex::update_sensors_() {
  this->current_sensor_ = 0;
  auto sensor = sensors_[current_sensor_];
  auto read_sensor = AsyncWintexCommand{
    .cmd = WintexCommandType::READ_VOLATILE,
    .payload = read_payload(sensor->get_address(), sensor->get_length()),
    .callback = [this](WintexResponse response) {
      return this->handle_sensors_(response);
    }
  };
  queue_command_(read_sensor);
}

optional<AsyncWintexCommand> Wintex::handle_sensors_(WintexResponse response) {
  uint32_t address = (response.data[0] << 16) | (response.data[1] << 8) | response.data[2];
  uint8_t length = response.data[3];
  const uint8_t *data = &response.data[4];
  if (sensors_.size() == 0) {
    ESP_LOGV(TAG, "No sensors to update");
    return {};
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
/*
  if (current_sensor_  == sensors_.size()) {
    current_sensor_ = 0;
    return;
  }
*/
  auto sensor = sensors_[current_sensor_];
  auto read_sensor = AsyncWintexCommand{
    .cmd = WintexCommandType::READ_VOLATILE,
    .payload = read_payload(sensor->get_address(), sensor->get_length()),
    .callback = [this](WintexResponse response) {
      return this->handle_sensors_(response);
    }
  };
  queue_command_(read_sensor);
  return {};
}

void Wintex::send_command_now_(AsyncWintexCommand command) {
  last_command_timestamp_ = millis();
  if (this->current_command_.has_value()) {
    ESP_LOGE(TAG, "Sending command while another is in progress, this may not end well!!");
  }
  this->current_command_ = command;

  uint8_t len = (uint8_t)(command.payload.size()) + 3;

  ESP_LOGV(TAG, "%d: Sending Wintex: CMD=0x%02X DATA=[%s] lock=%u", last_command_timestamp_, static_cast<uint8_t>(command.cmd),
           format_hex_pretty(command.payload).c_str(), this->command_queue_lock_);

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
  if (!this->current_command_.has_value() && !command_queue_.empty()) {
    uint32_t delay = millis() - last_command_timestamp_;
    if (delay < COMMAND_DELAY)
      return;
    send_command_now_(command_queue_.front());
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

void Wintex::queue_command_(AsyncWintexCommand command) {
  command_queue_.push_back(command);
  process_command_queue_();
}

}  // namespace wintex
}  // namespace esphome

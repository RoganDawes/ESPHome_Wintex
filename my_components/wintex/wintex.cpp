#include "wintex.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace wintex {

static const char *TAG = "wintex";
static const int COMMAND_DELAY = 50;

void Wintex::setup() {
  this->set_interval("poll", 1000, [this] { 
    this->send_command_(WintexCommand{.cmd = WintexCommandType::READ_VOLATILE, .payload = std::vector<uint8_t>{0x00, 0x04, 0xf8, 0x20}}); 
  });
  this->send_command_(WintexCommand{.cmd = WintexCommandType::SESSION, .payload = std::vector<uint8_t>{udl_.begin(), udl_.end()}});
}

void Wintex::loop() {
  while (this->available()) {
    uint8_t c;
    this->read_byte(&c);
    this->handle_char_(c);
  }
  process_command_queue_();
}

void Wintex::dump_config() {
  ESP_LOGCONFIG(TAG, "Wintex:");
  if (this->init_state_ != WintexInitState::INIT_DONE) {
    ESP_LOGCONFIG(TAG, "  Configuration will be reported when setup is complete. Current init_state: %u",
                  static_cast<uint8_t>(this->init_state_));
    ESP_LOGCONFIG(TAG, "  If no further output is received, confirm that this is a supported Wintex device.");
    return;
  }
  for (auto &info : this->datapoints_) {
    if (info.type == WintexDatapointType::ZONE_STATUS)
      ESP_LOGCONFIG(TAG, "  Datapoint %u: zone (value: %s)", info.id, hexencode(info.value_raw).c_str());
    else
      ESP_LOGCONFIG(TAG, "  Datapoint %u: unknown", info.id);
  }
  ESP_LOGCONFIG(TAG, "  Product: '%s'", this->product_.c_str());
}

bool Wintex::validate_message_() {
  uint32_t at = this->rx_message_.size() - 1;
  auto *data = &this->rx_message_[0];
  
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
           hexencode(data + 2, length - 3).c_str(), static_cast<uint8_t>(this->init_state_), rx_checksum, calc_checksum);
    return false;
  }

  // valid message
  const uint8_t *message_data = data + 2;
  ESP_LOGV(TAG, "Received Wintex: CMD=0x%02X DATA=[%s] INIT_STATE=%u", static_cast<uint8_t>(command),
           hexencode(message_data, length - 3).c_str(), static_cast<uint8_t>(this->init_state_));
  this->handle_command_(command, message_data, length - 3);

  // return false to reset rx buffer
  return false;
}

void Wintex::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);
  if (!this->validate_message_()) {
    this->rx_message_.clear();
  }
}

void Wintex::handle_command_(WintexResponseType command, const uint8_t *buffer, size_t len) {
  switch (command) {
    case WintexResponseType::HANGUP:
      ESP_LOGV(TAG, "Logged out");
      this->init_state_ = WintexInitState::UNAUTH;
      this->send_command_(WintexCommand{.cmd = WintexCommandType::SESSION, .payload = std::vector<uint8_t>{0x01, 0x09, 0x06, 0x07}});
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
          this->product_ = std::string(reinterpret_cast<const char *>(buffer), len);
          ESP_LOGV("Successful authentication, product [%s]", this->product_.c_str());
          this->init_state_ = WintexInitState::AUTH;
          this->send_command_(WintexCommand{.cmd = WintexCommandType::READ_VOLATILE, .payload = std::vector<uint8_t>{0x00, 0x05, 0x00, 0x20}});
        } else {
          this->product_ = R"({"p":"INVALID"})";
        }
      } else if (len == 0x09) {
        ESP_LOGV("Heartbeat response [%s]", hexencode(buffer, len).c_str());
      }
      break;
      case WintexResponseType::READ_VOLATILE:
        if (len == 0x24 && buffer[0] == 0x00 && buffer[1] == 0x04 && buffer[2] == 0xf8 && buffer[3] == 0x20) {
          for (uint8_t zone = 0; zone < 0x20; zone++) {
            this->handle_zone_(zone + 1, buffer[4+zone]);
          }
        }
        break;
    default:
      ESP_LOGE(TAG, "Invalid command (0x%02X) received", static_cast<uint8_t>(command));
  }
}

void Wintex::handle_zone_(uint8_t zone, uint8_t status) {
  WintexDatapoint datapoint{};
  datapoint.id = zone;
  datapoint.type = WintexDatapointType::ZONE_STATUS;
  datapoint.value_bool = (status & 0x01); // 1 if active

  // Update internal datapoints
  bool found = false;
  for (auto &other : this->datapoints_) {
    if (other.id == datapoint.id) {
      other = datapoint;
      found = true;
    }
  }
  if (!found) {
    this->datapoints_.push_back(datapoint);
  }

  // Run through listeners
  for (auto &listener : this->listeners_)
    if (listener.datapoint_id == datapoint.id)
      listener.on_datapoint(datapoint);
}

void Wintex::send_raw_command_(WintexCommand command) {
  this->last_command_timestamp_ = millis();
  uint8_t len = (uint8_t)(command.payload.size()) + 3;

  ESP_LOGV(TAG, "Sending Wintex: CMD=0x%02X DATA=[%s] INIT_STATE=%u", static_cast<uint8_t>(command.cmd),
           hexencode(command.payload).c_str(), static_cast<uint8_t>(this->init_state_));

  this->write_array({len, (uint8_t) command.cmd});
  if (!command.payload.empty())
    this->write_array(command.payload.data(), command.payload.size());

  uint8_t checksum = len + (uint8_t) (command.cmd);
  for (auto &data : command.payload)
    checksum += data;
  checksum ^= 0xFF;

  this->write_byte(checksum);
}

void Wintex::process_command_queue_() {
  uint32_t delay = millis() - this->last_command_timestamp_;
  // Left check of delay since last command in case theres ever a command sent by calling send_raw_command_ directly
  if (delay > COMMAND_DELAY && !command_queue_.empty()) {
    this->send_raw_command_(command_queue_.front());
    this->command_queue_.erase(command_queue_.begin());
  }
}

void Wintex::send_command_(WintexCommand command) {
  command_queue_.push_back(command);
  process_command_queue_();
}

void Wintex::send_empty_command_(WintexCommandType command) {
  send_command_(WintexCommand{.cmd = command, .payload = std::vector<uint8_t>{}});
}

optional<WintexDatapoint> Wintex::get_datapoint_(uint8_t datapoint_id) {
  for (auto &datapoint : this->datapoints_)
    if (datapoint.id == datapoint_id)
      return datapoint;
  return {};
}

void Wintex::register_listener(uint8_t datapoint_id, const std::function<void(WintexDatapoint)> &func) {
  auto listener = WintexDatapointListener{
      .datapoint_id = datapoint_id,
      .on_datapoint = func,
  };
  this->listeners_.push_back(listener);

  // Run through existing datapoints
  for (auto &datapoint : this->datapoints_)
    if (datapoint.id == datapoint_id)
      func(datapoint);
}

}  // namespace wintex
}  // namespace esphome

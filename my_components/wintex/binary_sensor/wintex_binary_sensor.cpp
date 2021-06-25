#include "esphome/core/log.h"
#include "wintex_binary_sensor.h"

namespace esphome {
namespace wintex {

static const char *TAG = "wintex.binary_sensor";

void WintexBinarySensor::setup() {
  this->parent_->register_listener(this->sensor_id_, [this](WintexDatapoint datapoint) {
    ESP_LOGV(TAG, "MCU reported binary sensor %u is: %s", datapoint.id, ONOFF(datapoint.value_bool));
    this->publish_state(datapoint.value_bool);
  });
}

void WintexBinarySensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Wintex Binary Sensor:");
  ESP_LOGCONFIG(TAG, "  Binary Sensor has datapoint ID %u", this->sensor_id_);
}

}  // namespace wintex
}  // namespace esphome

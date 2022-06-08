#include "esphome/core/log.h"
#include "wintex_binary_sensor.h"

namespace esphome {
namespace wintex {

static const char *TAG = "wintex.binary_sensor";

void WintexBinarySensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Wintex Binary Sensor:");
  // ESP_LOGCONFIG(TAG, "  Binary Sensor '%s' has ID %u", this->get_name().c_str(), this->sensor_id_);
}

void WintexBinarySensor::update_wintex(const uint8_t *memory) {

}
}  // namespace wintex
}  // namespace esphome

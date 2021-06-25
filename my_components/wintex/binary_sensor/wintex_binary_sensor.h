#pragma once

#include "esphome/core/component.h"
#include "esphome/components/wintex/wintex.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace wintex {

class WintexBinarySensor : public binary_sensor::BinarySensor, public Component {
 public:
  void setup() override;
  void dump_config() override;
  void set_sensor_id(uint8_t sensor_id) { this->sensor_id_ = sensor_id; }

  void set_wintex_parent(Wintex *parent) { this->parent_ = parent; }

 protected:
  Wintex *parent_;
  uint8_t sensor_id_{0};
};

}  // namespace wintex
}  // namespace esphome

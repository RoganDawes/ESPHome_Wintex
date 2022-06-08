#pragma once

#include "esphome/core/component.h"
#include "esphome/components/wintex/wintex.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace wintex {

class WintexBinarySensor : public WintexSensorBase, public binary_sensor::BinarySensor, public Component {
 public:
  virtual ~WintexBinarySensor() {}
  void setup() override;
  void dump_config() override;
  void set_wintex_binary_sensor(uint32_t address, uint8_t length, uint8_t offset, uint8_t mask) { 
    set_wintex_address(address, length, offset);
    this->mask_ = mask;
  }
  virtual void update_wintex(const uint8_t *memory) override;
  void set_wintex_parent(Wintex *parent) { this->parent_ = parent; }

 protected:
  Wintex *parent_;
  uint8_t mask_{0};
};

}  // namespace wintex
}  // namespace esphome

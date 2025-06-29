#pragma once
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace ads7830 {

class ADS7830Sensor : public PollingComponent, public sensor::Sensor, public i2c::I2CDevice {
 public:
  void set_channel(uint8_t channel) { channel_ = channel; }
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  uint8_t channel_{0};
};

}  // namespace ads7830
}  // namespace esphome
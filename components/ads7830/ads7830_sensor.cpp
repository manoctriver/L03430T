#include "ads7830_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ads7830 {

static const char *const TAG = "ads7830.sensor";

void ADS7830Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ADS7830 sensor...");
  ESP_LOGCONFIG(TAG, "ADS7830 Sensor I2C Address: 0x%02X, Channel: %u", this->address_, this->channel_);
  
  // Test communication by reading from the assigned channel
  uint8_t cmd = 0x84 | ((channel_ & 0x07) << 4);
  auto write_result = this->write(&cmd, 1);
  if (write_result != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write initial command to ADS7830 channel %u (error: %d)", this->channel_, write_result);
    this->status_set_warning();
    return;
  }
  
  // Give some time for conversion (ADS7830 can be slow)
  delay_microseconds_safe(10000);
  
  uint8_t value;
  auto read_result = this->read(&value, 1);
  if (read_result == i2c::ERROR_OK) {
    ESP_LOGCONFIG(TAG, "ADS7830 sensor channel %u setup successful, initial read: %u", this->channel_, value);
    this->status_clear_warning();
  } else {
    ESP_LOGW(TAG, "Failed to read initial value from ADS7830 channel %u (error: %d)", this->channel_, read_result);
    this->status_set_warning();
  }
}

void ADS7830Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "ADS7830 Sensor:");
  ESP_LOGCONFIG(TAG, "  Channel: %u", this->channel_);
  LOG_SENSOR("  ", "ADS7830", this);
  LOG_I2C_DEVICE(this);
}

void ADS7830Sensor::update() {
  uint8_t cmd = 0x84 | ((channel_ & 0x07) << 4);
  auto write_result = this->write(&cmd, 1);
  if (write_result != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write command to ADS7830 channel %u (error: %d)", this->channel_, write_result);
    this->status_set_warning();
    return;
  }
  
  // Give some time for conversion (ADS7830 needs time to settle)
  delay_microseconds_safe(10000);
  
  uint8_t value;
  auto read_result = this->read(&value, 1);
  if (read_result == i2c::ERROR_OK) {
    float voltage = value / 255.0f * 5.0f; // Adjust 5.0f to your reference voltage
    ESP_LOGD(TAG, "Channel %u: Raw=%u, Voltage=%.3fV", this->channel_, value, voltage);
    this->publish_state(voltage);
    this->status_clear_warning();
  } else {
    ESP_LOGW(TAG, "Failed to read from ADS7830 channel %u (error: %d)", this->channel_, read_result);
    this->status_set_warning();
  }
}

}  // namespace ads7830
}  // namespace esphome
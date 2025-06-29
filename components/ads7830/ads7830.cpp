#include "ads7830.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ads7830 {

static const char *const TAG = "ads7830";

void ADS7830Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ADS7830...");
  ESP_LOGCONFIG(TAG, "ADS7830 I2C Address: 0x%02X", this->address_);
  
  // Test communication by trying to write a command and read a value
  uint8_t cmd = 0x84; // Channel 0, single-ended, internal ref, AD on
  auto write_result = this->write(&cmd, 1);
  if (write_result != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write test command to ADS7830 (error: %d)", write_result);
    this->mark_failed();
    return;
  }
  
  // Give some time for conversion
  delay_microseconds_safe(5000);
  
  uint8_t value;
  auto read_result = this->read(&value, 1);
  if (read_result != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read test value from ADS7830 (error: %d)", read_result);
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "ADS7830 setup successful, test read: %u", value);
}

void ADS7830Component::dump_config() {
  ESP_LOGCONFIG(TAG, "ADS7830:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with ADS7830 failed!");
  }
}

}  // namespace ads7830
}  // namespace esphome

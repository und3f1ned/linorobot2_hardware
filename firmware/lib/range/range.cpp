// range.cpp
#include "range.h"
#include <Arduino.h>
#include <micro_ros_utilities/string_utilities.h>

TFMiniI2C tfmini;

TFMiniI2C::TFMiniI2C() {}

bool TFMiniI2C::init() {
  // Initialize I2Cdev
  i2c_dev.initialize();

  // Configure range message
  range_msg_.header.frame_id =
      micro_ros_string_utilities_set(range_msg_.header.frame_id, "tfmini_link");
  range_msg_.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_msg_.field_of_view = FOV;
  range_msg_.min_range = MIN_RANGE;
  range_msg_.max_range = MAX_RANGE;

  // Test connection
  uint8_t test_data;
  return i2c_dev.readByte(TFMINI_I2C_ADDR, 0x00, &test_data);
}

uint16_t TFMiniI2C::getDistance() {
  uint16_t distance;
  uint8_t buffer[2];

  if (i2c_dev.readBytes(TFMINI_I2C_ADDR, TFMINI_DISTANCE_REG, 2, buffer)) {
    distance = (buffer[1] << 8) | buffer[0];
    return distance;
  }
  return 0;
}

uint16_t TFMiniI2C::getSignalStrength() {
  uint16_t strength;
  uint8_t buffer[2];

  if (i2c_dev.readBytes(TFMINI_I2C_ADDR, TFMINI_STRENGTH_REG, 2, buffer)) {
    strength = (buffer[1] << 8) | buffer[0];
    return strength;
  }
  return 0;
}

sensor_msgs__msg__Range TFMiniI2C::getRangeMessage() {
  uint16_t distance = getDistance();
  uint16_t strength = getSignalStrength();

  if (strength > 100 && distance < 1200) {
    range_msg_.range = distance / 100.0; // Convert to meters
  } else {
    range_msg_.range = +INFINITY;
  }

  return range_msg_;
}

// Global functions to maintain compatibility with existing code
void initRange() { tfmini.init(); }

sensor_msgs__msg__Range getRange() { return tfmini.getRangeMessage(); }

#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/range.h>

#define TFMINI_I2C_ADDR 0x10
#define FOV (2.0 * 0.0174533) // 2 degrees to radians
#define MIN_RANGE 0.30        // 30cm
#define MAX_RANGE 12.00       // 12m

sensor_msgs__msg__Range range_msg_;

uint16_t readDistance() {
  Wire.beginTransmission(TFMINI_I2C_ADDR);
  Wire.write(0x01); // Distance data register
  Wire.endTransmission();

  if (Wire.requestFrom(TFMINI_I2C_ADDR, 2) == 2) {
    uint16_t distance = Wire.read() | (Wire.read() << 8);
    return distance;
  }
  return 0;
}

uint16_t readStrength() {
  Wire.beginTransmission(TFMINI_I2C_ADDR);
  Wire.write(0x03); // Signal strength register
  Wire.endTransmission();

  if (Wire.requestFrom(TFMINI_I2C_ADDR, 2) == 2) {
    uint16_t strength = Wire.read() | (Wire.read() << 8);
    return strength;
  }
  return 0;
}

sensor_msgs__msg__Range getRange() {
  uint16_t distance = readDistance();
  uint16_t strength = readStrength();

  if (strength > 100 && distance < 1200) {
    range_msg_.range = distance / 100.0; // Convert to meters
  } else {
    range_msg_.range = +INFINITY;
  }

  range_msg_.field_of_view = FOV;
  range_msg_.min_range = MIN_RANGE;
  range_msg_.max_range = MAX_RANGE;

  return range_msg_;
}

void initRange() {
  range_msg_.header.frame_id =
      micro_ros_string_utilities_set(range_msg_.header.frame_id, "tfmini_link");
  range_msg_.radiation_type = sensor_msgs__msg__Range__INFRARED;
}

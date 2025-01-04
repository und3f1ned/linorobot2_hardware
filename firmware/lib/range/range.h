// range.h
#ifndef RANGE_H
#define RANGE_H

#include "I2Cdev.h"
#include <sensor_msgs/msg/range.h>

#define TFMINI_I2C_ADDR 0x10
#define TFMINI_DISTANCE_REG 0x01
#define TFMINI_STRENGTH_REG 0x03

class TFMiniI2C {
public:
  TFMiniI2C();
  bool init();
  uint16_t getDistance();
  uint16_t getSignalStrength();
  sensor_msgs__msg__Range getRangeMessage();

private:
  I2Cdev i2c_dev;
  sensor_msgs__msg__Range range_msg_;
  const float FOV = 2.0 * 0.0174533; // 2 degrees to radians
  const float MIN_RANGE = 0.10;      // 10cm
  const float MAX_RANGE = 12.00;     // 12m
};

void initRange();

sensor_msgs__msg__Range getRange();

extern TFMiniI2C tfmini;

#endif

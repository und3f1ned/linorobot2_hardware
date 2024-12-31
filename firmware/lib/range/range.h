#ifndef RANGE_H
#define RANGE_H

#include <Wire.h>
#include <sensor_msgs/msg/range.h>

#define TFMINI_I2C_ADDR 0x10
#define TFMINI_DISTANCE_REG 0x01
#define TFMINI_STRENGTH_REG 0x03

sensor_msgs__msg__Range getRange();
void initRange();
uint16_t readDistance();
uint16_t readStrength();

#endif

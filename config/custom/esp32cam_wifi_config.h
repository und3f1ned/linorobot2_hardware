#ifndef ESP32_CAM_CONFIG_H
#define ESP32_CAM_CONFIG_H

#define LED_PIN 33

#define LINO_BASE DIFFERENTIAL_DRIVE
// #define LINO_BASE SKID_STEER
// #define LINO_BASE MECANUM

// #define USE_BTS7960_MOTOR_DRIVER
// #define USE_GENERIC_2_IN_MOTOR_DRIVER
#define USE_GENERIC_1_IN_MOTOR_DRIVER
// #define USE_ESC_MOTOR_DRIVER

#define K_P 0.6
#define K_I 0.8
#define K_D 0.5

#define ACCEL_COV                                                              \
  { 0.01, 0.01, 0.01 }
#define GYRO_COV                                                               \
  { 0.001, 0.001, 0.001 }
#define ORI_COV                                                                \
  { 0.01, 0.01, 0.01 }
#define MAG_COV                                                                \
  { 1e-12, 1e-12, 1e-12 }
#define POSE_COV                                                               \
  { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
#define TWIST_COV                                                              \
  { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

#define MOTOR_MAX_RPM 39
#define MAX_RPM_RATIO 0.85
#define MOTOR_OPERATING_VOLTAGE 6
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 12
#define COUNTS_PER_REV1 4180
#define COUNTS_PER_REV2 4180
#define COUNTS_PER_REV3 4180
#define COUNTS_PER_REV4 4180
#define WHEEL_DIAMETER 0.062
#define LR_WHEELS_DISTANCE 0.1
#define PWM_BITS 10
#define PWM_FREQUENCY 20000
// #define SERVO_BITS 12
// #define SERVO_FREQ 50

#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

// ENCODER PINS
#define MOTOR1_ENCODER_A 0
#define MOTOR1_ENCODER_B 1

#define MOTOR2_ENCODER_A 3
#define MOTOR2_ENCODER_B 16

// #define MOTOR3_ENCODER_A 32
// #define MOTOR3_ENCODER_B 27

// #define MOTOR4_ENCODER_A 26
// #define MOTOR4_ENCODER_B 25

#ifdef USE_BTS7960_MOTOR_DRIVER
#define MOTOR1_PWM -1
#define MOTOR1_IN_A 12
#define MOTOR1_IN_B 13

#define MOTOR2_PWM -1
#define MOTOR2_IN_A 2
#define MOTOR2_IN_B 4

// #define MOTOR3_PWM -1
// #define MOTOR3_IN_A 13
// #define MOTOR3_IN_B 12

// #define MOTOR4_PWM -1
// #define MOTOR4_IN_A 4
// #define MOTOR4_IN_B 23

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX
#endif

#define AGENT_IP                                                               \
  { 192, 168, 1, 100 }
#define AGENT_PORT 8888
#define WIFI_AP_LIST                                                           \
  {                                                                            \
    {"WIFI_SSID", "WIFI_PASSWORD"}, { NULL }                                   \
  }
#define WIFI_MONITOR 2
#define USE_ARDUINO_OTA
#define USE_SYSLOG
#define SYSLOG_SERVER                                                          \
  { 192, 168, 1, 100 }
#define SYSLOG_PORT 514
#define DEVICE_HOSTNAME "esp32cam_wifi"
#define APP_NAME "hardware"
// #define USE_LIDAR_UDP
// #define LIDAR_RXD 14
// #define LIDAR_SERIAL 1
// #define LIDAR_BAUDRATE 230400
// #define LIDAR_SERVER \
//   { 192, 168, 1, 100 }
// #define LIDAR_PORT 8889
#define BAUDRATE 921600
#define SDA_PIN 14
#define SCL_PIN 15
#define NODE_NAME "esp32cam_wifi"

// #define BATTERY_PIN 33
// #ifdef USE_ADC_LUT
// const int16_t ADC_LUT[4096] = {};
// #define BATTERY_ADJUST(v) (ADC_LUT[v] * (3.3 / 4096 * (33 + 10) / 10 * 1.0))
// #else
// #define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * (33 + 10) / 10))
// #endif

// #define BATTERY_DIP 0.98
#define USE_SHORT_BRAKE

#define BOARD_INIT                                                             \
  {                                                                            \
    Wire.begin(SDA_PIN, SCL_PIN);                                              \
    Wire.setClock(400000);                                                     \
  }

#ifdef USE_SYSLOG
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      syslog(LOG_ERR, "%s RCCHECK failed %d", __FUNCTION__, temp_rc);          \
      return false;                                                            \
    }                                                                          \
  }
#else
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      flashLED(3);                                                             \
      return false;                                                            \
    }                                                                          \
  }
#endif

#endif

#ifndef ESP32_CAM_CONFIG_H
#define ESP32_CAM_CONFIG_H

#define LED_PIN 33  // ESP32-CAM built-in LED
#define FLASH_PIN 4 // ESP32-CAM Flash LED

// Camera pins
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_BTS7960_MOTOR_DRIVER

// Modified motor pins for ESP32-CAM compatibility
#define MOTOR1_PWM -1
#define MOTOR1_IN_A 12
#define MOTOR1_IN_B 13

#define MOTOR2_PWM -1
#define MOTOR2_IN_A 14
#define MOTOR2_IN_B 15

#define PWM_BITS 10
#define PWM_FREQUENCY 20000
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

// Network configuration
#define AGENT_IP                                                               \
  { 192, 168, 1, 100 }
#define AGENT_PORT 8888
#define WIFI_AP_LIST                                                           \
  {                                                                            \
    {"WIFI_SSID", "WIFI_PASSWORD"}, { NULL }                                   \
  }
#define DEVICE_HOSTNAME "esp32cam"
#define APP_NAME "hardware"

// I2C pins
#define SDA_PIN 26
#define SCL_PIN 27

#define NODE_NAME "esp32cam"

// Camera settings
#define XCLK_FREQ 20000000
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA
#define CAMERA_JPEG_QUALITY 12
#define CAMERA_FB_COUNT 1

// Initialize board
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

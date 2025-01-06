// camera.cpp
#include "camera.h"

#define CAMERA_MODEL_AI_THINKER
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

Camera::Camera() : camera_initialized_(false) {}

void Camera::setup(rcl_node_t *node_handle) {
  static const char *camera_topic = "/image/compressed";

  // Initialize publisher
  constexpr int qos = 10;
  RCCHECK(rclc_publisher_init_default(
      &publisher_, node_handle,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
      camera_topic));

  initCamera();
}

void Camera::initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // QVGA resolution
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12; // 0-63, lower means higher quality
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  camera_initialized_ = true;
}

bool Camera::captureFrame() {
  if (!camera_initialized_) {
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    return false;
  }

  msg_.format.data = (unsigned char *)"jpeg";
  msg_.format.size = strlen("jpeg");
  msg_.data.data = fb->buf;
  msg_.data.size = fb->len;

  esp_camera_fb_return(fb);
  return true;
}

void Camera::publish() {
  if (captureFrame()) {
    RCSOFTCHECK(rcl_publish(&publisher_, &msg_, NULL));
  }
}

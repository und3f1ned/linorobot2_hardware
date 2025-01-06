// camera.h
#ifndef CAMERA_H
#define CAMERA_H

#include "Arduino.h"
#include "esp_camera.h"
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/compressed_image.h>

class Camera {
private:
  sensor_msgs__msg__CompressedImage msg_;
  rcl_publisher_t publisher_;
  rclc_executor_t executor_;
  rclc_support_t support_;
  rcl_allocator_t allocator_;
  rcl_node_t node_;
  bool camera_initialized_;

  void initCamera();
  bool captureFrame();

public:
  Camera();
  void setup(rcl_node_t *node_handle);
  void publish();
};

#endif

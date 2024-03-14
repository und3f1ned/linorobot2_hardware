// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "battery.h"
#include "range.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"
#include "pwm.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;
MAG mag;
unsigned total_motors = 4;

void setup()
{
    Serial.begin(BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
#ifdef BOARD_INIT // board specific setup
    BOARD_INIT;
#endif

    initPwm();
    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();
    initWifis();
    initOta();

    imu.init();
    mag.init();
    initBattery();
    initRange();

    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }
    motor1_encoder.getRPM();
    motor2_encoder.getRPM();
    motor3_encoder.getRPM();
    motor4_encoder.getRPM();

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE;
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    delay(2000);
}

const unsigned ticks = 20;
const float dt = ticks * 0.001;
const unsigned run_time = 1000; // 1s
const unsigned buf_size = run_time / ticks * 4;
Kinematics::velocities buf[buf_size];
float imu_max_acc = 0, imu_min_acc = 0;
unsigned idx = 0;
void record(unsigned n) {
    unsigned i;
    for (i = 0; i < n; i++, idx++) {
        float rpm1 = motor1_encoder.getRPM();
        float rpm2 = motor2_encoder.getRPM();
        float rpm3 = motor3_encoder.getRPM();
        float rpm4 = motor4_encoder.getRPM();
        imu_msg = imu.getData();
        float imu_acc_x = imu_msg.linear_acceleration.x;
        if (imu_acc_x > imu_max_acc) imu_max_acc = imu_acc_x;
        if (imu_acc_x < imu_min_acc) imu_min_acc = imu_acc_x;

        if (idx < buf_size)
            buf[idx] = kinematics.getVelocities(rpm1, rpm2, rpm3, rpm4);
        delay(ticks);
        runWifis();
        runOta();
    }
}

void dump_record(void) {
    float max_vel = 0, min_vel = 0, max_acc = 0, min_acc = 0;
    float dist = 0;
    for (idx = 0; idx < buf_size; idx++) {
        float vel_x = buf[idx].linear_x;
        float vel_y = buf[idx].linear_y;
        float vel_z = buf[idx].angular_z;
        if (vel_x > max_vel) max_vel = vel_x;
        if (vel_x < min_vel) min_vel = vel_x;
        Serial.printf("%04d VEL %6.2f %6.2f m/s  %6.2f rad/s\n",
            idx, vel_x, vel_y, vel_z);
        syslog(LOG_INFO, "%04d VEL %6.2f %6.2f m/s  %6.2f rad/s",
            idx, vel_x, vel_y, vel_z);
    }
    for (idx = 0; idx < buf_size; idx++) {
        unsigned prev = idx ? (idx -1) : 0;
        float acc_x = (buf[idx].linear_x - buf[prev].linear_x) / dt;
        float acc_y = (buf[idx].linear_y - buf[prev].linear_y) / dt;
        float acc_z = (buf[idx].angular_z - buf[prev].angular_z) / dt;
        if (acc_x > max_acc) max_acc = acc_x;
        if (acc_x < min_acc) min_acc = acc_x;
        Serial.printf("%04d ACC %6.2f %6.2f m/s2 %6.2f rad/s2\n",
            idx, acc_x, acc_y, acc_z);
        syslog(LOG_INFO, "%04d ACC %6.2f %6.2f m/s2 %6.2f rad/s2",
            idx, acc_x, acc_y, acc_z);
    }
    for (idx = buf_size / 4; idx < buf_size / 2; idx++)
        dist += buf[idx].linear_x * dt;
    for (idx = 0; idx < buf_size; idx++) {
        if (buf[idx].linear_x > max_vel * 0.9) break;
    }
    Serial.printf("MAX VEL %6.2f %6.2f m/s\n", max_vel, min_vel);
    Serial.printf("MAX ACC %6.2f %6.2f m/s2\n", max_acc, min_acc);
    Serial.printf("IMU ACC %6.2f %6.2f m/s2\n", imu_max_acc, imu_min_acc);
    Serial.printf("time to 0.9x max vel %6.2f sec\n", idx * dt);
    Serial.printf("distance to stop %6.2f m\n", dist);
    syslog(LOG_INFO, "MAX VEL %6.2f %6.2f m/s", max_vel, min_vel);
    syslog(LOG_INFO, "MAX ACC %6.2f %6.2f m/s2", max_acc, min_acc);
    syslog(LOG_INFO, "IMU ACC %6.2f %6.2f m/s2", imu_max_acc, imu_min_acc);
    syslog(LOG_INFO, "time to 0.9x max vel %6.2f sec", idx * dt);
    syslog(LOG_INFO, "distance to stop %6.2f m", dist);
}

unsigned runs = 2;
void loop() {
    const int pwm_max = PWM_MAX;
    const int pwm_min = -pwm_max;

    while (runs) {
        runs--;
        idx = 0;
        // full speed forward
        digitalWrite(LED_PIN, HIGH);
        motor1_controller.spin(pwm_max);
        motor2_controller.spin(pwm_max);
        motor3_controller.spin(pwm_max);
        motor4_controller.spin(pwm_max);
        record(run_time / ticks);
        // stop
        digitalWrite(LED_PIN, LOW);
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        motor4_controller.spin(0);
        record(run_time / ticks);
        // full speed backward
        digitalWrite(LED_PIN, HIGH);
        motor1_controller.spin(pwm_min);
        motor2_controller.spin(pwm_min);
        motor3_controller.spin(pwm_min);
        motor4_controller.spin(pwm_min);
        record(run_time / ticks);
        // stop
        digitalWrite(LED_PIN, LOW);
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        motor4_controller.spin(0);
        record(run_time / ticks);
        // print result
        dump_record();
    }

    // idle
    delay(100);
    runWifis();
    runOta();
}

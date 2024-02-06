#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

#ifdef PCA_BASE
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
#endif

float pwm_freq = PWM_FREQUENCY;

void setPWMFreq(float freq)
{
    pwm_freq = freq;
#ifndef TEENSYDUINO
#ifdef PICO
    analogWriteFreq(freq);
#else
    analogWriteFrequency(freq);
#endif
#endif
#ifdef PCA_BASE
    pwm.setPWMFreq(freq);  // Set to whatever you like, we don't use it in this demo!
#endif
}

void initPwm()
{
#ifdef PCA_BASE
    pwm.begin();
#endif
    setPWMFreq(PWM_FREQUENCY);
    analogWriteResolution(PWM_BITS);
}

void setPin(int pin, int value)
{
    if (pin < 0) return;
#ifdef PCA_BASE
    if (pin >= PCA_BASE)
        pwm.setPin(pin - PCA_BASE, value);
#endif
    else
        analogWrite(pin, value);
}

void setLevel(int pin, int value)
{
    if (pin < 0) return;
#ifdef PCA_BASE
    if (pin >= PCA_BASE)
        pwm.setPin(pin - PCA_BASE, value ? PWM_MAX : 0);
#endif
    else
        digitalWrite(pin, value);
}

void setPulse(int pin, int value)
{
    if (pin < 0) return;
    int pulse = value / ((1000000 / pwm_freq) / (1 << PWM_BITS));
    setPin(pin, pulse);
}

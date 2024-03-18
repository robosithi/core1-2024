#ifndef SERVO_HH
#define SERVO_HH

#include <Arduino.h>
#include <M5Stack.h>
#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pwm;

void initServo();
void setServoPulse(uint8_t n, double pulse);
void servo_angle_write(uint8_t n, int Angle);

#endif
#ifndef ARM_CONTROL_HH
#define  ARM_CONTROL_HH

#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include <cybergear_controller.hh>
#include <Wire.h>
#include "servo.hh"


// extern const int arm_shoulder_detect_pin;
// extern const int arm_elbow_detect_pin;
// extern int arm_reseted;

extern CybergearController controller;

void init_arm();

int do_arm_reset();
int arm_reset_check(int do_flag);
void move_arm(int emergency ,int arm_button);

void move_arm_servo(int mode);
#endif
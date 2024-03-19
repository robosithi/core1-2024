#ifndef NECK_CONTROL_HH
#define NECK_CONTROL_HH


#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include <cybergear_controller.hh>
#include <Wire.h>
#include"common.hh"

extern const float SPIN_NECK_GEARRATIO;
extern const float CYBERGEAR_GEARRATIO;//Cybergearのギア比
extern const float SPIN_OMEGA;//SPIN時の回転速度。
//SPIN_OMEGA*(MECHANUM_LENGTH_X+MECHANUM_LENGTH_Y)/MECHANUM_TIRE_Rを30以下にしないと、タイヤのサイバーギアが間に合わない。

extern CybergearController controller;
class NeckMotorController{
public:
    NeckMotorController(){
        motor_id = 123;
        ref_position = 0;
        ref_speed = 0;
        pos_gain = 1;
        max_speed = PI;
        motor_status = {0,0,0,0,0,0,0,0,0};
        front_threshould = -PI;
        pos_offset_sum = 0;
    }
    void init();
    void position_control(float pos);
    void speed_control(float speed);
    float get_pos(){
        return (motor_status.position + pos_offset_sum)/(SPIN_NECK_GEARRATIO);
    }
    float get_speed(){
        return ref_speed;
    }
    uint8_t get_motor_id(){
        return motor_id;
    }
    void set_status(MotorStatus sta){
        if(sta.motor_id == motor_id){
            motor_status = sta;
        }
    }
    float loop(int emergency,int spin_switch);
    float nearest_angle (float angle,float offset = 0.0);
private:
    uint8_t motor_id;
    float ref_position;
    float ref_speed;
    float pos_gain;
    float max_speed;
    float pos_offset_sum;
    MotorStatus motor_status;
    // float get_neck_target(float position);
    float front_threshould;
};



#endif
#include "neck_control.hh"

const float SPIN_OMEGA = PI;//SPIN時の回転速度。
//SPIN_OMEGA*(MECHANUM_LENGTH_X+MECHANUM_LENGTH_Y)/MECHANUM_TIRE_Rを30以下にしないと、タイヤのサイバーギアが間に合わない。
const float SPIN_NECK_GEARRATIO = 2; //首のギア比。減速で1より大きくする

void NeckMotorController::init(){
//   controller.init(motor_id, MODE_SPEED, &CAN0);
  controller.set_mech_position_to_zero(motor_id);
}
void NeckMotorController::position_control(float pos){

}
void NeckMotorController::speed_control(float speed){
    if(speed > max_speed){
        speed = max_speed;
    }
    if(speed < -max_speed){
        speed = -max_speed;
    }
    controller.send_speed_command(motor_id,speed * SPIN_NECK_GEARRATIO);
}

float NeckMotorController::nearest_angle (float angle,float offset){
  while(angle-offset<=PI){
    angle -= 2*PI;
  }
  while(angle-offset>=-PI){
    angle += 2*PI;
  }
  return angle;
}

float NeckMotorController::get_neck_target(float position){
  float gear = nearest_angle(position/2.0);
  int rotate_num = int(position/2.0);
  if(gear> PI/2.0){
    rotate_num ++;
  }else if(gear<0 && gear>=-PI/2.0){
    rotate_num ++;
  }
  return 2.0*PI*float(rotate_num);
}
/// @brief neckMotorを動かすために毎周期呼ぶ関数
/// @param emergency ///緊急停止フラグ。撃破や無線断絶時に使用
/// @param spin_switch ///SPINさせるかどうかのスイッチ。ボタン入力を想定
/// @return 回転速度をFloatで返す。0の場合もある。
float NeckMotorController::loop(int emergency,int spin_switch){
    static float ret=0.0;
    if(emergency){
        speed_control(0.0);
        ret = 0.0;
        return 0;
    }
    if(spin_switch){
        if(motor_status.position>=0.0){
            speed_control(SPIN_OMEGA);
            ret = SPIN_OMEGA;
        }else{
            speed_control(-SPIN_OMEGA);
            ret = -SPIN_OMEGA;
        }
    }else{
        if(abs(nearest_angle(motor_status.position))<front_threshould){
            speed_control(0);
            ret = 0;
        }else{
            speed_control(ret);            
        }
    }
    return ret;
}
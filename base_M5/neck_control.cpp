#include "neck_control.hh"

const float SPIN_OMEGA = PI/2.0;//SPIN時の回転速度。max = PI
//SPIN_OMEGA*(MECHANUM_LENGTH_X+MECHANUM_LENGTH_Y)/MECHANUM_TIRE_Rを30以下にしないと、タイヤのサイバーギアが間に合わない。
const float SPIN_NECK_GEARRATIO = 2.0; //首のギア比。減速で1より大きくする
// const float CYBERGEAR_GEARRATIO = 7.75;//Cybergearのギア比

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
  while(angle-offset>=PI){
    angle -= 2*PI;
  }
  while(angle-offset<=-PI){
    angle += 2*PI;
  }
  return angle;
}

// float NeckMotorController::get_neck_target(float position){
//   float gear = nearest_angle(position/(SPIN_NECK_GEARRATIO));
//   int rotate_num = int(position/((SPIN_NECK_GEARRATIO)));
//   if(gear> PI/2.0){
//     rotate_num ++;
//   }else if(gear<0 && gear>=-PI/2.0){
//     rotate_num ++;
//   }
//   return 2.0*PI*float(rotate_num);
// }

void NeckMotorController::interrupt(){


}

/// @brief neckMotorを動かすために毎周期呼ぶ関数
/// @param emergency ///緊急停止フラグ。撃破や無線断絶時に使用
/// @param spin_switch ///SPINさせるかどうかのスイッチ。ボタン入力を想定
/// @return 回転速度をFloatで返す。0の場合もある。
float NeckMotorController::loop(int emergency,int spin_switch){
    static float ret=0.0;
    old_spin_switch = spin_switch;
    static float old_angle = 0.0;

    if(emergency){
      ret = 0.0;
      return 0;
    }

    if(abs(motor_status.position)>2*PI){
      //2PI毎に値をリセット・それを積算して絶対角度を取得する
      pos_offset_sum += motor_status.position;
      controller.set_mech_position_to_zero(motor_id);
    }
    if(emergency){
        //speed_control(0.0);
        ret = 0.0;
        return 0;
    }
    if(spin_switch){
        if(abs(ret)<0.01){
          if(get_pos()>=0.0){
            //speed_control(SPIN_OMEGA);
            ret = - SPIN_OMEGA;
          }else{
              //speed_control(-SPIN_OMEGA);
            ret = SPIN_OMEGA;
          }
        }
        old_angle = nearest_angle(get_pos()*2.0);
    }else{
      //M5.Lcd.printf("angle_distance = %f\n",nearest_angle(get_pos()*2.0));
      // if(abs(nearest_angle(get_pos()*2.0)<PI/8)){
      if(abs(nearest_angle(get_pos()*2.0)<PI/8)||((nearest_angle(get_pos()*2.0)*old_angle)<0&&(nearest_angle(get_pos()*2.0)*old_angle)>-1)){
        //2をかけるのは前後をOKにするため。
        //１個前との掛け算が０を下回っているとき、正負が入れ替わっている。-piとpiのマタギでないことを確認するための閾値
          //speed_control(0);
          ret = 0;
      }else{
          //speed_control(ret);            
      }
    }
    return ret;
}
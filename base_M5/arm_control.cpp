#include "arm_control.hh"

//アームの姿勢検出用スイッチピン設定
const int arm_shoulder_detect_pin = 1;
const int arm_elbow_detect_pin = 3;
int arm_reseted = 0;


std::vector<uint8_t> position_motor_ids = {122, 121};//position controll list
std::vector<float> old_position = {0.0f, 0.0f};
std::vector<float> ref_position = {0.0f, 0.0f};
std::vector<float> zero_position = {0.0f, 0.0f};

const float mt0_motor_bend_angle = 0;
const float mt0_motor_extend_angle = PI/2;
const float mt1_motor_bend_angle = -PI/2.0;
const float mt1_motor_extend_angle = 0;
const float arm_reset_speed = 0.02;


void init_arm(){
  controller.init(position_motor_ids, MODE_POSITION, &CAN0);
    
  //アーム初期位置リセット用ボタン
  pinMode(arm_shoulder_detect_pin, INPUT_PULLUP);
  pinMode(arm_elbow_detect_pin, INPUT_PULLUP);
}


void set_res_pos_fromDiff(int id,float diff){
    ref_position[id]= old_position[id]+diff;
}

int arm_reset_status = 0;
int do_arm_reset(){
  switch(arm_reset_status){
    case 0://shoulder reset
      set_res_pos_fromDiff(0,-arm_reset_speed);
      set_res_pos_fromDiff(1,0.0);
      if(digitalRead(arm_shoulder_detect_pin)==1){
        arm_reset_status++;
        zero_position[0] = old_position[0];
        // controller.set_mech_position_to_zero(motion_motor_ids[1]);
        set_res_pos_fromDiff(0,0);
      }
      break;
    case 1://elbow reset
      set_res_pos_fromDiff(0,0.0);
      set_res_pos_fromDiff(1,arm_reset_speed);
      if(digitalRead(arm_elbow_detect_pin)==1){
        arm_reset_status++;
        zero_position[1] = old_position[1];
        // controller.set_mech_position_to_zero(motion_motor_ids[1]);
        set_res_pos_fromDiff(1,0);
        return 1;
      }
      break;
    default:
      arm_reseted = 1;
      return 1;
      break;
  }
  controller.send_position_command(position_motor_ids,ref_position);
  old_position[0] = ref_position[0];
  old_position[1] = ref_position[1];
  
  M5.Lcd.printf("DoingArmReset ref 0=%f zero 0=%f ref 1=%f zero 1=%f\n",ref_position[0],zero_position[0],ref_position[1],zero_position[1]);
  return 0;
}

int arm_reset_check(int do_flag = 0){
  int return_num = arm_reseted;
  if(arm_reseted ==0){
    if(do_flag==1){
      return_num = do_arm_reset();
    }
  }
  return return_num;
}

const int arm_servo_bend  = 140;
const int arm_servo_extend = 86;
const int arm_servo_wait = 40;
const int hand_servo_open = 80;
const int hand_servo_hold = 64;
const int camera_servo_bend = 80;//64~99
const int camera_servo_extend = 70;
const int arm_servo = 6;
const int hand_servo = 7;
const int camera_servo = 8;
void move_arm_servo(int mode){
  switch (mode){
    case 0://extend
      servo_angle_write(arm_servo,arm_servo_extend);
      servo_angle_write(hand_servo,hand_servo_open);
      servo_angle_write(camera_servo,camera_servo_extend);
      break;
    case 1://bend
      servo_angle_write(arm_servo,arm_servo_bend);
      servo_angle_write(hand_servo,hand_servo_open);
      servo_angle_write(camera_servo,camera_servo_bend);
      break;
      
    case -1:
    default:
      servo_angle_write(arm_servo,arm_servo_wait);
      servo_angle_write(hand_servo,hand_servo_open);
      servo_angle_write(camera_servo,camera_servo_bend);    
      break;
  }

}

void move_arm(int emergency ,int arm_button){
  static int old_button = 0;
  static int old_arm_status = -1;
  
  if(arm_reset_check(!emergency&&arm_button)){//リセットがまだの場合、実行される
      //armがリセット済みであった場合
    if(old_button ==0 && arm_button == 1){
      if(old_arm_status !=0){//伸ばす。extend
        ref_position[0] = mt0_motor_extend_angle-zero_position[0];
        ref_position[1] = mt1_motor_extend_angle-zero_position[1];
        old_arm_status = 0;
      }else{//曲げるBend
        ref_position[0] = mt0_motor_bend_angle-zero_position[0];
        ref_position[1] = mt1_motor_bend_angle-zero_position[1];
        old_arm_status = 1;
      }
      
    }
    controller.send_position_command(position_motor_ids,ref_position);
  }
  old_button = arm_button;
  move_arm_servo(old_arm_status);
}
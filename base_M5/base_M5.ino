// build as software for M5Core
// you need install M5 board, M5 Device, cybergear_controller 

#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include <cybergear_controller.hh>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


/**
 * Servo Module settings
*/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVOMIN \
    102  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX \
    512  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN \
    500  // This is the rounded 'minimum' microsecond length based on the
#define USMAX \
    2500  // This is the rounded 'maximum' microsecond length based on the
#define SERVO_FREQ \
    50  // Analog servos run at ~50 Hz updates  模拟伺服以 ~50 Hz 更新运行


/**
 * @brief Init can interface
 */
void init_can();

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;

std::vector<uint8_t> motor_ids = {127, 126, 125, 124};//speed controll list
std::vector<float> speeds = {0.0f, 0.0f, 0.0f, 0.0f};

std::vector<uint8_t> motion_motor_ids = {123, 122, 121};//speed controll list
std::vector<CybergearMotionCommand> motion_commands = {{0.0f,0.0f,0.0f,0.0f,DEFAULT_VELOCITY_KP},
                                                       {0.0f,0.0f,0.0f,0.0f,DEFAULT_VELOCITY_KP},
                                                       {0.0f,0.0f,0.0f,0.0f,DEFAULT_VELOCITY_KP}};

// init cybergeardriver
CybergearController controller = CybergearController(MASTER_CAN_ID);

#define DATA_BUFFER_SIZE (50)
char input[DATA_BUFFER_SIZE] = {0};
unsigned char decoded_data[8] = {0};
int input_num = 0;
bool a_button = false;
bool b_button = false;
bool x_button = false;
bool y_button = false;

bool spin_switch = false;
bool shoot_ready = false;


const uint8_t nBits_forPWM = 8; // PWMに使用するビット数　n=1～16[bit]
                            //PWM周波数の最大値 Maxfreq=80000000.0/2^n[Hz]=312500[Hz]
const uint8_t PWM_CH = 2;   // PWMチャンネル
const uint8_t SHOOT_DIR_1 = 25;   // DIRチャンネル
const uint8_t SHOOT_PIN = 2;  // PWM出力に使用するGPIO PIN番号
const int PWM_Values = 70; //デューティ 実質的な射出速度 70->約27%
                            //MaxDuty=2^n (nBits_forPWM)  DutyRatio = Duty/MaxDuty
const double PWM_Frequency = 2000.0;
                            // PWM周波数 Maxfreq=80000000.0/2^n[Hz]

//撃破時動作停止系変数宣言
const int destroyed_pin = 21; //撃破信号入力ピン　、Inverce（0だと撃破されている状態）
int destroyed_flag = 0; //撃破されていたか、フラグ。切り替え時のみの処理に使用

//アームの姿勢検出用スイッチピン設定
const int arm_shoulder_detect_pin = 23;
const int arm_elbow_detect_pin = 19;
int arm_reseted = 0;

// controller.set_mech_position_to_zero();

//定時割り込み機能系設定
hw_timer_t * timer = NULL;
bool timer_flag = 0;


void setup()
{
  M5.begin();

  // init cybergear driver
  init_can();
  
  delay(1000);

  //表示設定
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0,2);


  // init position offset
  M5.Lcd.print("Init arm & neck motors ... ");
  controller.init(motion_motor_ids, MODE_MOTION, &CAN0);
  M5.Lcd.println("done");
  controller.enable_motors();

  controller.set_mech_position_to_zero(motion_motor_ids[0]);
  
  M5.Lcd.print("move to motor origin ... ");
  controller.send_motion_command(motion_motor_ids, motion_commands);
  delay(1000);
  M5.Lcd.println("done");

  Serial2.begin(115200, SERIAL_8N1, 16,17);  // Init serial port 2.

  
  // start bilateral mode
  M5.Lcd.print("starting mechanum speed controll ... ");
  controller.init(motor_ids, MODE_SPEED, &CAN0);
  // controller.enable_motors();
  M5.Lcd.println("done");
  controller.enable_motors();
  M5.Lcd.println("Start All Cybergears, done");
  
  //Servo module setup
    pwm.begin();
    pwm.setPWMFreq(50);

  //set Pin mode
  pinMode(SHOOT_PIN, OUTPUT); 
  pinMode(SHOOT_DIR_1, OUTPUT); 
  digitalWrite(SHOOT_DIR_1, HIGH);//射出モータの方向を変える

  //撃破時停止機能のための入力
  pinMode(destroyed_pin, INPUT_PULLUP);

  // //アーム初期位置リセット用ボタン
  // pinMode(arm_shoulder_detect_pin, INPUT_PULLUP);
  // pinMode(arm_elbow_detect_pin, INPUT_PULLUP);

  //コントローラ情報などリセット
  inital_data_set();
  // リセット
  // タイマ作成
  timer = timerBegin(0, 80, true);
  // タイマ割り込みサービス・ルーチン onTimer を登録
  timerAttachInterrupt(timer, &onTimer, true);
  // 割り込みタイミング(ms)の設定
  timerAlarmWrite(timer, 1000, true);
  // タイマ有効化
  timerAlarmEnable(timer);
  Serial2.flush();
  M5.Lcd.println("System all green!");
}

const float MAX_SPEED = 25.0; //最高速度変更用,rad/s,CyberGear的MAXは30
const float MAX_OMEGA = 30.0; //最高回転速度変更用 上から見た回転速度でrad/s,上から見て左回り
const float SPIN_OMEGA = 15.0;//SPIN時の回転速度。??以下にしないと、首のサイバーギアが間に合わない。
const float SPIN_NECK_GEARRATIO = 1.0; //首のギア比。減速で1より大きくする
const float MECHANUM_LENGTH_X = 0.4;
const float MECHANUM_LENGTH_Y = 0.6;


int right_holizontal_zero_pos=135;
int left_holizontal_zero_pos=132;
int right_vertical_zero_pos=136;
int left_vertical_zero_pos=135;

unsigned long  wireless_get_time = 3000;

float calculate_ratio(unsigned char joint_input, int initial_pos,float dir = 1){
  int joint_diff = joint_input - initial_pos;
  // Serial.println(joint_diff);
  if(joint_diff >= 0){
    return dir * (float)(  (joint_diff / float(255.0 - initial_pos)));
  }else{
    return dir * (float)(  (joint_diff / float(initial_pos)));
  }
}

void onTimer(){//タイマー割込みで実行される関数
  if(millis()-wireless_get_time>2000){//無線断絶停止の閾値設定
    timer_flag = 1;
  }
}

int get_controller_data(){
  int flag = 0;
  int get_cnt = 0;
  if(get_cnt = Serial2.available()){
    for(int cnt = 0;cnt<get_cnt;cnt++){
      char data = Serial2.read();
      if(data == '\r'){
        input_num = 0;
        // 不要なデータをスキップ
        char *p = input;
        while (*p != ':') {
          p++;
        }
        p++;

        // 7つのchar型のデータを格納
        for (int i = 0; i < 8; i++) {
          decoded_data[i] = strtol(p, &p, 16);
          if (*p == ',') {
            p++;
          }
        }

        if(decoded_data[0]!=0){
          flag = 1;
        }
        
        a_button    = (decoded_data[5] & 0x01)? 1:0;
        y_button    = (decoded_data[5] & 0x02)? 1:0;
        spin_switch = (decoded_data[5] & 0x04)? 1:0;
        shoot_ready = (decoded_data[5] & 0x08)? 1:0;
        b_button    = (decoded_data[5] & 0x10)? 1:0;
        x_button    = (decoded_data[5] & 0x20)? 1:0;

      }else{
        input[input_num] = data;
        input_num++;
        if(input_num>DATA_BUFFER_SIZE){
          // Serial.write(input,50);
          input_num = 0;
        }
      }
    }
  }
  return flag;

}
void set_motion_speed_controll(CybergearMotionCommand &cmd,float vel){
  cmd.kp = 0;
  cmd.kd = DEFAULT_VELOCITY_KP;
  cmd.effort = 0;
  cmd.velocity = vel;
  cmd.position = 0;
}
void set_motion_position_controll(CybergearMotionCommand &cmd,float pos){
  cmd.kp = DEFAULT_POSITION_KP;
  cmd.kd = 0;
  cmd.effort = 0;
  cmd.velocity = 0;
  cmd.position = pos;
}
const float arm_reset_speed = 0.5;
int do_arm_reset(){
  set_motion_speed_controll((motion_commands[1]),arm_reset_speed);
  controller.send_motion_command(motion_motor_ids[1],motion_commands[1]);

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
// std::vector<uint8_t> position_motor_ids = {122, 121};//speed controll list
// std::vector<float> cyber_positions = {0.0f, 0.0f};
//DEFAULT_POSITION_KP
// /**
//  * @brief Cybegear motion command struct
//  */
// struct CybergearMotionCommand
// {
//   float position;   //!< target position
//   float velocity;   //!< target velocity (rad/sec)
//   float effort;     //!< target effort
//   float kp;         //!< motion control kp
//   float kd;         //!< motion control kd
// };

void setServoPulse(uint8_t n, double pulse) {
    double pulselength;
    pulselength = 1000000;  // 1,000,000 us per second
    pulselength /= 50;      // 50 Hz
    // Serial.print(pulselength);
    // Serial.println(" us per period");
    pulselength /= 4096;  // 12 bits of resolution
    // Serial.print(pulselength);
    // Serial.println(" us per bit");
    pulse *= 1000;
    pulse /= pulselength;
    // Serial.println(pulse);
    pwm.setPWM(n, 0, pulse);
}

void servo_angle_write(uint8_t n, int Angle) {
    double pulse = Angle;
    pulse        = pulse / 90 + 0.5;
    setServoPulse(n, pulse);
}

/// @brief 射出モータのON/OFFを切り替える関数
void shoot_ready_set(int emergency = 0){
  static int old_pin = 0;
  if(emergency){
    shoot_ready = 0;
  }
  if(old_pin!=shoot_ready){
    // チャンネルと周波数の分解能を設定
    ledcSetup(PWM_CH, PWM_Frequency, nBits_forPWM);
    // PWM出力ピンとチャンネルの設定
    ledcAttachPin(SHOOT_PIN, PWM_CH);
    // デューティーの設定と出力開始
    if(shoot_ready){
      ledcWrite(PWM_CH, PWM_Values);
    }else{
      ledcWrite(PWM_CH, 0);
    }
    old_pin = shoot_ready;
  }
}

const int servo_ready_angle_0 = 70;
const int servo_shoot_angle_0 = 30;
const int servo_ready_angle_1 = 55;
const int servo_shoot_angle_1 = 90;
const int servo_wait_cnt_max = 7; //射出スピードを設定。制御周期×この数＝左右1回ずつ射出する周期

/// @brief 射出用サーボをコントロールする
void shoot_servo_controll(int emergency = 0){
  static int old_pattarn = -1;
  if(emergency){
    if(old_pattarn !=0){
      old_pattarn = 0;
      servo_angle_write(0,servo_ready_angle_0);
      servo_angle_write(1,servo_ready_angle_1);
    }
    return;
  }

  static int servo_cnt = 0; 
  if(a_button){
    if(servo_cnt < servo_wait_cnt_max/2){
      if(old_pattarn!=1){
        old_pattarn = 1;
        //charge 0, shoot 1
        servo_angle_write(0,servo_ready_angle_0);
        servo_angle_write(1,servo_shoot_angle_1);
      }
      servo_cnt ++;
    }else if(servo_cnt < servo_wait_cnt_max){
      if(old_pattarn!=2){
        old_pattarn = 2;
        //shoot 0, charge 1
        servo_angle_write(0,servo_shoot_angle_0);
        servo_angle_write(1,servo_ready_angle_1);
      }
      servo_cnt++;
    }else{
      servo_cnt = 0;
    }
  }else{
    //charge both
    if(old_pattarn !=0){
      old_pattarn = 0;
      servo_angle_write(0,servo_ready_angle_0);
      servo_angle_write(1,servo_ready_angle_1);
      //  servo_cnt = 0; //to shoot evenly, do not reset cnt;
    }
  }
}
void inital_data_set(){
  decoded_data[2] = left_holizontal_zero_pos;
  decoded_data[1] = left_vertical_zero_pos;
  decoded_data[4] = right_holizontal_zero_pos;
  decoded_data[3] = right_vertical_zero_pos;
  decoded_data[5] = 0;

}


bool check_destroyed_stop(){
  bool flag = 0;
  if(flag = (digitalRead(destroyed_pin)==0)){
    if(destroyed_flag==0){
      controller.reset_motors();
      wireless_get_time = 0;
    }
  }else{
    if(destroyed_flag==1){
      controller.enable_motors();
      wireless_get_time = millis();
    }
  }
  return flag;
}

float nearest_angle (float angle,float target = 0.0){
  while(angle-target<=PI){
    angle -= 2*PI;
  }
  while(angle-target>=-PI){
    angle += 2*PI;
  }
  return angle;
}

float get_neck_target(float position){
  float gear = nearest_angle(position/2.0);
  int rotate_num = int(position/2.0);
  if(gear> PI/2.0){
    rotate_num ++;
  }else if(gear<0 && gear>=-PI/2.0){
    rotate_num ++;
  }
  return 2.0*PI*float(rotate_num);
}

void loop()
{
  // update m5 satatus
  M5.update();

  if(check_destroyed_stop()){
    M5.Lcd.println("Destroyed detected!");
  }

  // update and get motor data
  std::vector<MotorStatus> status_list;
  if ( controller.process_can_packet() ) {
    controller.get_motor_status(status_list);
  }
  
  // int stick_x_raw = analogReadMilliVolts(STICK_X_INPUT);
  // int stick_y_raw = analogReadMilliVolts(STICK_Y_INPUT);
  // const float offset_x = 1.1;
  // const float offset_y = 1.23;
  // float target_x = 2.0 * (float (stick_x_raw)/3000.0 ) - offset_x;
  // float target_y = -1*(2.0 * (float (stick_y_raw)/3000.0 ) - offset_y);
  // float target_omega = 0.0; 

  
  if(y_button){//Stickのイニシャライズ
    left_holizontal_zero_pos = decoded_data[2];
    left_vertical_zero_pos = decoded_data[1];
    right_holizontal_zero_pos = decoded_data[4];
    right_vertical_zero_pos = decoded_data[3];
  }


  float target_x = MAX_SPEED * calculate_ratio(decoded_data[2],left_holizontal_zero_pos);
  float target_y = MAX_SPEED * calculate_ratio(decoded_data[1],left_vertical_zero_pos,-1);
  float target_omega = MAX_OMEGA * calculate_ratio(decoded_data[4],right_holizontal_zero_pos);
  
  // Serial.printf("%d,%d,%d,%d,%d,%d,%d\n"
  // ,decoded_data[0],decoded_data[1],decoded_data[2],decoded_data[3]
  // ,decoded_data[4],decoded_data[5],decoded_data[6]);

  //首の角度を検出
  float neck_angle = 0;
  float raw_neck_angle = 0;
  MotorStatus neck_status;
  if(controller.get_motor_status(motion_motor_ids[0],neck_status)){
    raw_neck_angle = neck_status.position;
    neck_angle = nearest_angle(raw_neck_angle/2.0);
  }
  M5.Lcd.printf("neck check %d ,pos = %f ,raw = %f\n",neck_angle, raw_neck_angle);
  // int find_id = 0;
  // for(find_id= 0;find_id<status_list.size();++find_id){
  //   if(status_list[find_id].motor_id == motion_motor_ids[0]){
  //     neck_angle = nearest_angle(status_list[find_id].position/2.0);
  //     break;
  //   }
  // }
  // if(status_list.size()==0 || status_list[find_id].motor_id != motion_motor_ids[0]){
  //   find_id = -1;
  // }
  // M5.Lcd.printf("neck_angle %f status_list.size = %d\n",neck_angle,status_list.size());

  //add spin speed
  if(spin_switch){
    target_omega += SPIN_OMEGA;
    // set_motion_speed_controll(motion_commands[0],SPIN_NECK_GEARRATIO * SPIN_OMEGA);
    set_motion_position_controll(motion_commands[0],0.0);

  }else{
    set_motion_position_controll(motion_commands[0],0.0);
    // set_motion_position_controll(motion_commands[0],get_neck_target(raw_neck_angle));
    // if(find_id != -1){
    //   set_motion_position_controll(motion_commands[0],0.0);
    //   // set_motion_position_controll(motion_commands[0],get_neck_target(raw_neck_angle));
    //   M5.Lcd.printf("neck id check %d ,pos = %f \n",status_list[find_id].motor_id,status_list[find_id].position);
    // }else{
    //   set_motion_speed_controll(motion_commands[0],0.0f);
    // }
  }
  M5.Lcd.printf("Spin_ON kp = %f, kd = %f, effort = %f, position = %f, velocity = %f \n",
    motion_commands[0].kp,motion_commands[0].kd,motion_commands[0].effort,motion_commands[0].position,motion_commands[0].velocity);

    //calc each motor speed
  // {0,1,2,3} = {RrR, FrR, FrL, RrL} x-> right
  speeds[0] =   target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[1] = - target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[2] = - target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[3] =   target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;

  controller.send_speed_command(motor_ids, speeds);
  controller.send_motion_command(motion_motor_ids,motion_commands);
  shoot_ready_set(wireless_get_time == 0);
  shoot_servo_controll(wireless_get_time == 0);


  int get_data_flag = 0;
  while(!(get_data_flag =get_controller_data())&&(!timer_flag||wireless_get_time==0)){
  }
  if(timer_flag==1){
    wireless_get_time = 0; 
  }
  if(get_data_flag){
    wireless_get_time = millis();
    timer_flag = 0;
  }
  //受信できていないと推定されるとき
  if(wireless_get_time == 0){
    inital_data_set();
  }

  
  //表示部
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0,2);
  M5.Lcd.printf("Stick : %3d, %3d ,%3d,%3d \n",decoded_data[1],decoded_data[2],decoded_data[3],decoded_data[4]);
  M5.Lcd.printf("Speed : %.3f, %.3f ,%.3f \n",target_x,target_y,target_omega);
  M5.Lcd.printf("Motor : %.3f, %.3f, %.3f, %.3f \n",speeds[0],speeds[1],speeds[2],speeds[3]);
  M5.Lcd.printf("Button: %02x,A:%d, B:%d,X:%d, Y:%d, SPIN:%d, SHOOT:%d \n",decoded_data[5]
    ,a_button,b_button, x_button,y_button,spin_switch,shoot_ready);

  M5.Lcd.printf("disconnect_counter %d \n",wireless_get_time);

}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
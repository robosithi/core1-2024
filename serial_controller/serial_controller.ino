int initial_x_left=512;
int initial_y_left=512;
int initial_x_right=512;
int initial_y_right=512;

void setup() {
  // Digital input:
  // Button:
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  //Toggle Switch:
  pinMode(4,INPUT_PULLUP);
  //Joystick:
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);

  // Analog input:
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
}

void initialize_pos(int x_left, int y_left, int x_right, int y_right){
  if(digitalRead(3)==LOW){
    initial_x_left=x_left;
    initial_y_left=y_left;
    initial_x_right=x_right;
    initial_y_right=y_right;
  }
}

float calculate_ratio(int joint_input, int initial_pos){
  int joint_diff = joint_input - initial_pos;
  if(joint_diff >= 0){
    return joint_diff / (1023 - initial_pos);
  }else{
    return joint_diff / initial_pos;
  }
}

void view_serial(const char* state, int x, int y, int initial_x, int initial_y){
  char buf[44];
  sprintf(buf, "=== %s ===", state);
  Serial.println(buf);
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  initial_x: ");
  Serial.print(initial_x);
  Serial.print("  initial_y: ");
  Serial.println(initial_y);
}

void loop() {
  int x_left=analogRead(A0);
  int y_left=analogRead(A1);
  int x_right=analogRead(A2);
  int y_right=analogRead(A3);

  // initialize:
  initialize_pos(x_left, y_left, x_right, y_right);

  view_serial("left", x_left, y_left, initial_x_left, initial_y_left);
  view_serial("right", x_right, y_right, initial_x_right, initial_y_right);

  bool shot_sw=digitalRead(2);
  bool roll_mode=digitalRead(4);
  Serial.print("shot_sw: ");
  Serial.print(shot_sw);
  Serial.print("  roll_mode: ");
  Serial.println(roll_mode);

  float ratio_x_left = calculate_ratio(x_left, initial_x_left);
  float ratio_y_left = calculate_ratio(y_left, initial_y_left);
  float ratio_x_right = calculate_ratio(x_right, initial_x_right);
  float ratio_y_right = calculate_ratio(y_right, initial_y_right);
  view_serial("ratio", ratio_x_left, ratio_y_left, ratio_x_right, ratio_y_right);

  delay(100); //100ms=0.1sec
}

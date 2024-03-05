

int right_holizontal_zero_pos=512;
int left_holizontal_zero_pos=512;
int right_vertical_zero_pos=512;
int left_vertical_zero_pos=512;

#define A_BUTTON (2)
#define B_BUTTON (4)
#define X_BUTTON (3)
#define Y_BUTTON (5)
#define RESET_STICK_BUTTON (6)
#define TOGGLE_A (7)
#define TOGGLE_B (8)
#define RIGHT_STICK_BUTTON (9)
#define LEFT_STICK_BUTTON (10)

#define RIGHT_HOLIZONTAL_STICK (A0)
#define RIGHT_VERTICAL_STICK (A1)
#define LEFT_HOLIZONTAL_STICK (A2)
#define LEFT_VERTICAL_STICK (A3)


#define SHOT_BUTTON A_BUTTON
#define SPIN_SWITCH TOGGLE_A
#define LAUNCH_SWITCH TOGGLE_B

// #define TURN_SPEED_STICK RIGHT_HOLIZONTAL_STICK
// #define FROWARD_MOVE_STICK LEFT_VERTICAL_STICK
// #define HOLIZONTAL_MOVE_STICK LEFT_HOLIZONTAL_STICK

void setup() {
  // Digital input:
  // Button:
  pinMode(A_BUTTON,INPUT_PULLUP);
  pinMode(B_BUTTON,INPUT_PULLUP);
  pinMode(X_BUTTON,INPUT_PULLUP);
  pinMode(Y_BUTTON,INPUT_PULLUP);
  pinMode(RESET_STICK_BUTTON,INPUT_PULLUP);
  //Toggle Switch:
  pinMode(TOGGLE_A,INPUT_PULLUP);
  pinMode(TOGGLE_B,INPUT_PULLUP);
  //Joystick Button:
  pinMode(RIGHT_STICK_BUTTON,INPUT_PULLUP);
  pinMode(LEFT_STICK_BUTTON,INPUT_PULLUP);

  // // Analog input: pinmode設定がいらない
  // pinMode(RIGHT_HOLIZONTAL_STICK,INPUT);
  // pinMode(RIGHT_VERTICAL_STICK,INPUT);
  // pinMode(LEFT_HOLIZONTAL_STICK,INPUT);
  // pinMode(LEFT_VERTICAL_STICK,INPUT);

  Serial.begin(115200);
  Serial1.begin(115200);
}

void initialize_pos(){
  right_holizontal_zero_pos = analogRead(RIGHT_HOLIZONTAL_STICK);
  left_holizontal_zero_pos = analogRead(LEFT_HOLIZONTAL_STICK);
  right_vertical_zero_pos = analogRead(RIGHT_VERTICAL_STICK);
  left_vertical_zero_pos = analogRead(LEFT_VERTICAL_STICK);
}

signed char calculate_ratio(int joint_input, int initial_pos){
  int joint_diff = joint_input - initial_pos;
  Serial.println(joint_input);
  if(joint_diff >= 0){
    return (signed char)(127.0 * (joint_diff / float(1023.0 - initial_pos)));
  }else{
    return (signed char)(128.0 * (joint_diff / float(initial_pos)));
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

unsigned char make_button_data(){
  unsigned char data = 0;
  unsigned data_shift = 1;
  if(!digitalRead(A_BUTTON)){
    data = data | data_shift;
  }
  data_shift*=2;
  if(!digitalRead(B_BUTTON)){
    data = data | data_shift;
  }
  data_shift*=2;
  if(!digitalRead(X_BUTTON)){
    data = data | data_shift;
  }
  data_shift*=2;
  if(!digitalRead(Y_BUTTON)){
    data = data | data_shift;
  }
  data_shift*=2;
  if(!digitalRead(TOGGLE_A)){
    data = data | data_shift;
  }
  data_shift*=2;
  if(!digitalRead(TOGGLE_B)){
    data = data | data_shift;
  }
  data_shift*=2;
  if(!digitalRead(RIGHT_STICK_BUTTON)){
    data = data | data_shift;
  }
  data_shift*=2;
  if(!digitalRead(LEFT_STICK_BUTTON)){
    data = data | data_shift;
  }
  // data_shift*=2;
  return data;
}

void loop() {
  // int x_left=analogRead(A0);
  // int y_left=analogRead(A1);
  // int x_right=analogRead(A2);
  // int y_right=analogRead(A3);

  // initialize:
  if(!digitalRead(RESET_STICK_BUTTON)){
    initialize_pos();
  }

  // view_serial("left", x_left, y_left, initial_x_left, initial_y_left);
  // view_serial("right", x_right, y_right, initial_x_right, initial_y_right);

  // bool shot_sw=digitalRead(SHOT_BUTTON);
  // bool spin_mode=digitalRead(SPIN_SWITCH);
  // Serial.print("shot_sw: ");
  // Serial.print(shot_sw);
  // Serial.print("  roll_mode: ");
  // Serial.println(spin_mode);

// 各スティックと操作内容の対応関係はここで決まる
  signed char ratio_forward_move = calculate_ratio(analogRead(LEFT_VERTICAL_STICK), left_vertical_zero_pos);
  signed char ratio_holizontal_move = calculate_ratio(analogRead(LEFT_HOLIZONTAL_STICK), left_holizontal_zero_pos);
  signed char ratio_turn = calculate_ratio(analogRead(RIGHT_HOLIZONTAL_STICK), right_holizontal_zero_pos);
  signed char ratio_r_vertical = calculate_ratio(analogRead(RIGHT_VERTICAL_STICK), right_vertical_zero_pos);
  // view_serial("ratio", ratio_forward_move, ratio_holizontal_move, ratio_turn, ratio_r_vertical);

  //Make Button Data bite
  unsigned char button_bite = 0;
  button_bite = make_button_data();

  //Make data Packet
  char send_data [7]= {0};

  send_data[0] = ratio_forward_move;
  send_data[1] = ratio_holizontal_move;
  send_data[2] = ratio_turn;
  send_data[3] = ratio_r_vertical;
  send_data[4] = button_bite;
  send_data[5] = 0;
  send_data[6] = 0;

  for (int i=0;i<7;i++){
    char send_format[3] = {0};
    sprintf(send_format,"%02x",(unsigned char)(send_data[i]));
    Serial1.write(send_format,2);
    Serial.write(send_format,2);
    if(i<6){
      Serial1.write(',');
      Serial.write(',');
    }
  }
  Serial1.write("\r\n");
  Serial.write("\r\n");
  // Serial1.println();
  // Serial.println();
  // char debug_send_data[30] ={0};
  // sprintf(debug_send_data,"%3d,%3d,%3d,%3d,%3d,%3d,%3d",send_data[0],send_data[1],send_data[2],send_data[3],send_data[4],send_data[5],send_data[6]);
  // Serial.write(debug_send_data,30);
  // Serial.println();

  if(Serial1.available()){
    int ch = Serial1.read();
    Serial.write(ch);
  }
  delay(200); //100ms=0.1sec
}

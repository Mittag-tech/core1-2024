// build as software for M5Core
// you need install M5 board, M5 Device, cybergear_controller 

#include <Arduino.h>
#include <math.h>
#include <mcp_can.h>
#include <M5Stack.h>
#include <cybergear_controller.hh>

/**
 * @brief Init can interface
 */
void init_can();

// init MCP_CAN object
#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN CAN0(12);    // Set CS to pin 10

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;

std::vector<uint8_t> motor_ids = {127, 126, 125, 124};
std::vector<float> speeds = {0.0f, 0.0f, 0.0f, 0.0f};

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

// #define STICK_X_INPUT 35
// #define STICK_Y_INPUT 36


float offset_x = 1.1;
float offset_y = 1.23;

void setup()
{
  M5.begin();

  // init cybergear driver
  init_can();

  //表示設定
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0,2);

  // init position offset
  M5.Lcd.print("Init motors ... ");
  controller.init(motor_ids, MODE_POSITION, &CAN0);
  controller.enable_motors();
  M5.Lcd.println("done");

  M5.Lcd.print("move to motor origin ... ");
  controller.send_position_command(motor_ids, {0.0f, 0.0f});
  delay(1000);
  M5.Lcd.println("done");


  // start bilateral mode
  M5.Lcd.print("starting mechanum controll demo ... ");
  controller.init(motor_ids, MODE_SPEED, &CAN0);
  controller.enable_motors();
  M5.Lcd.println("done");

  Serial2.begin(115200, SERIAL_8N1, 16,17);  // Init serial port 2.
  
  // pinMode(STICK_X_INPUT, INPUT);
  // pinMode(STICK_Y_INPUT, INPUT);

}

const float MAX_SPEED = 1.0;
const float MECHANUM_LENGTH_X = 0.4;
const float MECHANUM_LENGTH_Y = 0.6;


int right_holizontal_zero_pos=135;
int left_holizontal_zero_pos=132;
int right_vertical_zero_pos=136;
int left_vertical_zero_pos=135;

float calculate_ratio(unsigned char joint_input, int initial_pos,float gain = 1){
  int joint_diff = joint_input - initial_pos;
  // Serial.println(joint_diff);
  if(joint_diff >= 0){
    return gain * (float)( MAX_SPEED* (joint_diff / float(255.0 - initial_pos)));
  }else{
    return gain * (float)( MAX_SPEED* (joint_diff / float(initial_pos)));
  }
}

int get_controller_data(){
  int flag = 0;
  if(Serial2.available()){
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
      
      a_button = (decoded_data[5] & 0x01)? 1:0;
      x_button = (decoded_data[5] & 0x02)? 1:0;
      spin_switch = (decoded_data[5] & 0x04)? 1:0;
      shoot_ready = (decoded_data[5] & 0x08)? 1:0;

    }else{
      input[input_num] = data;
      input_num++;
      if(input_num>DATA_BUFFER_SIZE){
        // Serial.write(input,50);
        input_num = 0;
      }
    }
  }
  return flag;

}

void loop()
{
  // update m5 satatus
  M5.update();


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

  while(!get_controller_data()){
  }


  float target_x = calculate_ratio(decoded_data[2],left_holizontal_zero_pos);
  float target_y = calculate_ratio(decoded_data[1],left_vertical_zero_pos,-1);
  float target_omega = calculate_ratio(decoded_data[4],right_holizontal_zero_pos);
  
  // Serial.printf("%d,%d,%d,%d,%d,%d,%d\n"
  // ,decoded_data[0],decoded_data[1],decoded_data[2],decoded_data[3]
  // ,decoded_data[4],decoded_data[5],decoded_data[6]);

  //calc each motor speed
  // {0,1,2,3} = {upper Right, upper left, lower left, lower right}
  speeds[0] =   target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[1] = - target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[2] = - target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[3] =   target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;

  controller.send_speed_command(motor_ids, speeds);

  //表示部
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0,2);
  M5.Lcd.printf("Stick : %3d, %3d ,%3d,%3d \n",decoded_data[1],decoded_data[2],decoded_data[3],decoded_data[4]);
  M5.Lcd.printf("Speed : %.3f, %.3f ,%.3f \n",target_x,target_y,target_omega);
  M5.Lcd.printf("Motor : %.3f, %.3f, %.3f, %.3f \n",speeds[0],speeds[1],speeds[2],speeds[3]);
  M5.Lcd.printf("Button: %02x,A:%d, X:%d, SPIN:%d, SHOOT:%d \n",decoded_data[5]
    ,a_button,x_button,spin_switch,shoot_ready);

  delay(30);
}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
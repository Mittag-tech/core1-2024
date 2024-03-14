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
         // 这是“最小”脉冲长度计数（共 4096 个）
#define SERVOMAX \
    512  // This is the 'maximum' pulse length count (out of 4096)
         // 这是“最大”脉冲长度计数（共 4096 个）
#define USMIN \
    500  // This is the rounded 'minimum' microsecond length based on the
         // minimum pulse of 102  这是基于 102 的最小脉冲的舍入“最小”微秒长度
#define USMAX \
    2500  // This is the rounded 'maximum' microsecond length based on the
          // maximum pulse of 512  这是基于 512 的最大脉冲的舍入“最大”微秒长度
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


const uint8_t nBits_forPWM = 8; // PWMに使用するビット数　n=1～16[bit]
                            //PWM周波数の最大値 Maxfreq=80000000.0/2^n[Hz]=312500[Hz]
const uint8_t PWM_CH = 2;   // PWMチャンネル
const uint8_t SHOOT_DIR_1 = 25;   // DIRチャンネル
const uint8_t SHOOT_PIN = 2;  // PWM出力に使用するGPIO PIN番号
const int PWM_Values = 70; //デューティ　デューティー比30%固定
//実質的な射出速度
                            //MaxDuty=2^n  DutyRatio = Duty/MaxDuty
const double PWM_Frequency = 2000.0;
                            // PWM周波数 Maxfreq=80000000.0/2^n[Hz]

// float offset_x = 1.1;
// float offset_y = 1.23;

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
  
  //Servo module setup
    pwm.begin();
    pwm.setPWMFreq(50);

  //set Pin mode
  pinMode(SHOOT_PIN, OUTPUT); 
  pinMode(SHOOT_DIR_1, OUTPUT); 
  digitalWrite(SHOOT_DIR_1, HIGH);//射出モータの方向を変える
  // pinMode(STICK_X_INPUT, INPUT);
  // pinMode(STICK_Y_INPUT, INPUT);

}

const float MAX_SPEED = 25.0; //最高速度変更用,rad/s,CyberGear的MAXは25?
const float MAX_OMEGA = 50.0; //最高回転速度変更用 m/s?
const float SPIN_OMEGA = 100.0;
const float MECHANUM_LENGTH_X = 0.4;
const float MECHANUM_LENGTH_Y = 0.6;


int right_holizontal_zero_pos=135;
int left_holizontal_zero_pos=132;
int right_vertical_zero_pos=136;
int left_vertical_zero_pos=135;

float calculate_ratio(unsigned char joint_input, int initial_pos,float dir = 1){
  int joint_diff = joint_input - initial_pos;
  // Serial.println(joint_diff);
  if(joint_diff >= 0){
    return dir * (float)(  (joint_diff / float(255.0 - initial_pos)));
  }else{
    return dir * (float)(  (joint_diff / float(initial_pos)));
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


void setServoPulse(uint8_t n, double pulse) {
    double pulselength;
    pulselength = 1000000;  // 1,000,000 us per second
    pulselength /= 50;      // 50 Hz
    Serial.print(pulselength);
    Serial.println(" us per period");
    pulselength /= 4096;  // 12 bits of resolution
    Serial.print(pulselength);
    Serial.println(" us per bit");
    pulse *= 1000;
    pulse /= pulselength;
    Serial.println(pulse);
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
const int servo_wait_cnt_max = 10;

/// @brief 射出用サーボをコントロールする
void shoot_servo_controll(int emergency = 0){
  if(emergency){
    servo_angle_write(0,servo_ready_angle_0);
    servo_angle_write(1,servo_ready_angle_1);
    return;
  }
  static int servo_cnt = 0; 
  if(a_button){
    if(servo_cnt < servo_wait_cnt_max/2){
      //charge 0, shoot 1
      servo_angle_write(0,servo_ready_angle_0);
      servo_angle_write(1,servo_shoot_angle_1);
      servo_cnt ++;
    }else if(servo_cnt < servo_wait_cnt_max){
      //shoot 0, charge 1
      servo_angle_write(0,servo_shoot_angle_0);
      servo_angle_write(1,servo_ready_angle_1);
      servo_cnt++;
    }else{
      servo_cnt = 0;
    }
  }else{
    //charge both
      servo_angle_write(0,servo_ready_angle_0);
      servo_angle_write(1,servo_ready_angle_1);
      //  servo_cnt = 0; //to shoot evenly, do not reset cnt;
  }
}
void inital_data_set(){
  decoded_data[2] = left_holizontal_zero_pos;
  decoded_data[1] = left_vertical_zero_pos;
  decoded_data[4] = right_holizontal_zero_pos;
  decoded_data[3] = right_vertical_zero_pos;
  decoded_data[5] = 0;

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

  int wireless_cnt = 0;
  while(!get_controller_data() ){
    wireless_cnt++;
    if(wireless_cnt>10000){
      wireless_cnt = -1;
      break;
    }
  }
  if(wireless_cnt == -1){
    inital_data_set();
  }

  if(x_button || y_button){
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

  //add spin speed
  if(spin_switch){
    target_omega += SPIN_OMEGA;
    // speed[4] = 0.5;
  }else{
    // speed[4] = 0.0;
  }

  //calc each motor speed
  // {0,1,2,3} = {upper Right, upper left, lower left, lower right}?
  speeds[0] =   target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[1] = - target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[2] = - target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[3] =   target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;

  controller.send_speed_command(motor_ids, speeds);
  shoot_ready_set();
  shoot_servo_controll();

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
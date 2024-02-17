// build as software for M5Core
// you need install M5 board, M5 Device, cybergear_controller 

// #include <cybergear_bridge.hh>
// #include <cybergear_bridge_packet.hh>
// #include <cybergear_controller.hh>
// #include <cybergear_driver.hh>
// #include <cybergear_driver_defs.hh>
// #include <ros2_logo.hh>
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
#define STICK_X_INPUT 35
#define STICK_Y_INPUT 36


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

  
  pinMode(STICK_X_INPUT, INPUT);
  pinMode(STICK_Y_INPUT, INPUT);

}

const float MAX_SPEED = 1.0;
const float MECHANUM_LENGTH_X = 0.4;
const float MECHANUM_LENGTH_Y = 0.6;

void loop()
{
  // update m5 satatus
  M5.update();

  controller.send_speed_command(motor_ids, speeds);

  // update and get motor data
  std::vector<MotorStatus> status_list;
  if ( controller.process_can_packet() ) {
    controller.get_motor_status(status_list);
  }

  
  int stick_x_raw = analogReadMilliVolts(STICK_X_INPUT);
  int stick_y_raw = analogReadMilliVolts(STICK_Y_INPUT);

  const float offset_x = 1.1;
  const float offset_y = 1.23;

  float target_x = 2.0 * (float (stick_x_raw)/3000.0 ) - offset_x;
  float target_y = -1*(2.0 * (float (stick_y_raw)/3000.0 ) - offset_y);
  float target_omega = 0.0;

  //calc each motor speed
  // {0,1,2,3} = {upper Right, upper left, lower left, lower right}
  speeds[0] =   target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[1] = - target_x + target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[2] = - target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;
  speeds[3] =   target_x - target_y + (MECHANUM_LENGTH_X + MECHANUM_LENGTH_Y) * target_omega;

  //表示部
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0,2);
  M5.Lcd.printf("Stick : %.3f, %.3f \n",target_x,target_y);
  M5.Lcd.printf("speed : %.3f, %.3f, %.3f, %.3f \n",speeds[0],speeds[1],speeds[2],speeds[3]);
  delay(20);
}

void init_can()
{
  CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);  // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
}
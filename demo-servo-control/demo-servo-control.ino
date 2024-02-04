#include <Servo.h>
Servo servo;
int x_value;
int y_value;
int input_value;

void setup() {
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(1, INPUT);

  servo.attach(10, 500, 2400);
  pinMode(10, OUTPUT);

  Serial.begin(9600);
}
void loop() {
  int initial_x=492;
  x_value=analogRead(A0);
  y_value=analogRead(A1);

  if (x_value-initial_x<0) {
    input_value-=(initial_x-x_value)*180/(1023-initial_x);
  } else {
    input_value+=(x_value-initial_x)*180/(1023-initial_x);
  }
  
  servo.write(input_value);
  Serial.print("X: ");
  Serial.print(x_value);
  Serial.print("  Y: ");
  Serial.println(y_value);
  delay(100);
}
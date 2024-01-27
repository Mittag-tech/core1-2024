//joystick 1のx, y軸
int x_value1;
int y_value1;

//joystick 2のx, y軸
int x_value2;
int y_value2;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  // pinMode(A2,INPUT);
  // pinMode(A3,INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  x_value1=analogRead(A0);
  y_value1=analogRead(A1);
  x_value2=analogRead(A2);
  y_value2=analogRead(A3);

  Serial.print("X1: ");
  Serial.print(x_value1);
  Serial.print("  Y1: ");
  Serial.print(y_value1);
  Serial.print("  x2: ");
  Serial.print(x_value2);
  Serial.print("  y2: ");
  Serial.println(y_value2);

  delay(100); //100ms=0.1sec
}

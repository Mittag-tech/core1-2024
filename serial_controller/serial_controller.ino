int max_val=1023;

void setup() {
  // Digital input:
  pinMode(2,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);

  // Analog input:
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
}

int initialize_pos(char degital_input, int analog_input){
  int initial_pos;
  if(digitalRead(degital_input)==LOW){
    initial_pos=analog_input;
  }
  return initial_pos;
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
  int initial_x_left=initialize_pos(12,x_left);
  int initial_y_left=initialize_pos(12,y_left);
  int initial_x_right=initialize_pos(13,x_right);
  int initial_y_right=initialize_pos(13,y_right);  

  view_serial("left", x_left, y_left, initial_x_left, initial_y_left);
  view_serial("right", x_right, y_right, initial_x_right, initial_y_right);

  bool shot_sw=digitalRead(2);
  Serial.print("shot_sw: ");
  Serial.println(shot_sw);

  delay(100); //100ms=0.1sec
}

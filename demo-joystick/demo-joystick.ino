//x軸方向の値
int x_value;
 
//y軸方向の値
int y_value;
 
//スイッチの状態を見る変数(押されていない:1 押されている:0)
bool switch_flag;
 
void setup() {
 
  //VRxに繋いだピン(A0)をを入力ピンとして設定する(デフォルト設定なので省略しても良い)
  pinMode(A0,INPUT);
 
  //VRyに繋いだピン(A1)をを入力ピンとして設定する(デフォルト設定なので省略しても良い)
  pinMode(A1,INPUT);
  
  //SWに繋いだピン(2)をプルアップ付きの入力ピンとして設定する
  pinMode(2,INPUT_PULLUP);
 
  //シリアル通信の初期処理を行う
  Serial.begin(9600);
}
 
void loop() {
  
  //x軸方向とy軸方向の値を取得する
  x_value=analogRead(A0);
  y_value=analogRead(A1);
 
  //スイッチの状態を取得する
  switch_flag=digitalRead(2);
 
  //x軸方向とy軸方向の値、スイッチの状態を表示する
  Serial.print("X: ");
  Serial.print(x_value);
  Serial.print("  Y: ");
  Serial.print(y_value);
  Serial.print("  SW: ");
  Serial.println(switch_flag);
 
  delay(100);
}
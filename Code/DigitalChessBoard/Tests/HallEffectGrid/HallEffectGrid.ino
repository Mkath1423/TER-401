#include Wire.h

const int power_pins[2] = {3, 4};
const int signal_pins[5] = {A1, A2, A3, A4, A5};

const int n_power_pins = 2;
const int n_signal_pins = 5;


void setup() {
  for(int i = 0; i != 2; i++){
    pinMode(power_pins[i], OUTPUT);
  }

  for(int i = 0; i != 5; i++){
    pinMode(signal_pins[i], INPUT);
  }

  Serial.begin(9600);
  Wire.begin()
}


int active_column = 0;

String command;
void loop(){
  if(Serial.available()){
    command = Serial.readString();
    command.trim();
    if(command == "rb"){
      CheckBoard();
    }
    else if(command.substring(0, 2) == "cp"){
      
    }

    command = "";
  }
}


void TransmitPixels(String command){
  Wire.beginTransmission(9)
  Wire.send(command);
  Wire.endTransmission();
  // transmit here
}

void CheckBoard(){
  int data[10] = {};
  
  for(int i = 0; i != n_power_pins; i++){
  
    SetColumnPower(i);
    for(int j = 0; j != n_signal_pins; j++){
      Serial.println(String(analogRead(signal_pins[j])) + " " + String(i) + String(j) + String(i*n_signal_pins + j));
      data[(i)*n_signal_pins + j] = analogRead(signal_pins[j]) < 40;      
    }
   
  }
  SetColumnPower(-1);
  SendSensorData(data);
}

void SetColumnPower(int col){
  for(int i = 0; i != n_power_pins; i++){
    digitalWrite(power_pins[i], i == col ? HIGH : LOW);
  }
}

void SendSensorData(int data[10]){
  String out="";
  for(int i = 0; i != n_power_pins*n_signal_pins; i++ ){
    out += String(data[i]);
  }
  Serial.println(out);
}

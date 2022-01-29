#include "Wire.h"

const int power_pins[2] = {2, 3};
const int signal_pins[5] = {A1, A2, A3, A4, A5};

const int n_cols = 2;
const int n_rows = 5;

String board[64] = {};

void setup() {
  for(int i = 0; i != 2; i++){
    pinMode(power_pins[i], OUTPUT);
  }

  for(int i = 0; i != 5; i++){
    pinMode(signal_pins[i], INPUT);
  }
  Wire.begin(); 
  Serial.begin(9600);
}

String command;
int active_column = 0;
void loop() {
  if(Serial.available()){
    // read incoming command
    command = Serial.readString();
    if(command.startsWith("rb")){
      // read the sensors and write the readings
      CheckBoard();
    }
    else if(command.startsWith("echo")){
      // write echo (used for serial warm up)
      Serial.println("echo");
    }
    else if(command.startsWith("sb")){
      // send light changes to the light handler
      String changes = command.substring(3);

     // parse data into CharArray
      char * buf;
      buf = (char *)malloc(changes.length()+1);
      changes.toCharArray(buf, changes.length()+1);

      
       // transmit to device #4
      Wire.beginTransmission(4);
      Wire.write(buf); 
      Wire.endTransmission();
    }
    
    command = "";
  }
  
  delay(100);
}

void CheckBoard(){
  String out = "";
  for(int i = 0; i != n_cols; i++){
    // give power to one column
    SetColumnPower(i);
    
    // read data from each sensor in the column
    for(int i = 0; i != n_rows; i++){
      out += String(analogRead(signal_pins[i]) < 100) + " ";
    }
    SetColumnPower(-1);
  }
  Serial.println(out);
}

void SetColumnPower(int col){
  for(int i = 0; i != n_cols; i++){
    digitalWrite(power_pins[i], i == col ? HIGH : LOW);
  }
}

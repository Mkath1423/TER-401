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
    command = Serial.readString();
    Serial.println(command);
    if(command.startsWith("rb")){
      Serial.println("Reading Board");
      Serial.println("-------------");
      CheckBoard();
    }
    else if(command.startsWith("sb")){
      Serial.println("Sending New Position");
       // transmit to device #4
      String changes = command.substring(4);
      char buf[changes.length()];
      
      changes.toCharArray(buf, changes.length());
      
      Serial.println(buf);
      for(int i = 0; i != buf.length(); i++){
        
        Serial.println(buf[i]);
      }
      //Wire.beginTransmission(4);
      //Wire.write(bytes);   // sends one byte  
      //Wire.endTransmission();    // stop transmitting
    }
    
    command = "";
  }
  
  delay(100);
}

void CheckBoard(){
  for(int i = 0; i != n_cols; i++){
    
    SetColumnPower(i);
    for(int i = 0; i != n_rows; i++){
      Serial.print(String(analogRead(signal_pins[i]) < 100) + " ");
    }
    Serial.println();
    SetColumnPower(-1);
  }
}

void SetColumnPower(int col){
  for(int i = 0; i != n_cols; i++){
    digitalWrite(power_pins[i], i == col ? HIGH : LOW);
  }
}

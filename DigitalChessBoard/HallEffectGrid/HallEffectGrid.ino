const int power_pins[2] = {2, 3};
const int signal_pins[5] = {A1, A2, A3, A4, A5};

const int n_cols = 2;
const int n_rows = 5;


void setup() {
  for(int i = 0; i != 2; i++){
    pinMode(power_pins[i], OUTPUT);
  }

  for(int i = 0; i != 5; i++){
    pinMode(signal_pins[i], INPUT);
  }

  Serial.begin(9600);
}


int active_column = 0;
void loop() {
  if(Serial.available()){
    Serial.readString();
    Serial.println("Reading Board");
    Serial.println("---------------------");
    CheckBoard();
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

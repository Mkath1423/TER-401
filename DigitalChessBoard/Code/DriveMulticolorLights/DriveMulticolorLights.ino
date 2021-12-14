/* RG Matrix Example   v.2 8/1/08  BroHogan
 * Demos 2 color 8x8 matrix driven by 2 MAX7821's
 */
#include "Arduino.h"                   // needed to compile with Rel. 0013 WHY?!
#include "LedControl.h"                 // to drive the matrix
#include "Wire.h"
#define ISR_FREQ 190     //190=650Hz    // Sets the speed of the ISR - LOWER IS FASTER
// prescaler is /128 -  125,000/ISR_FREQ+1 (i.e 249=500Hz, 190=650Hz)
// Tweaked depending on the overhead in the ISR & and other factors in the sketch
// Display will be slow if too FAST. Sketch will hang on delay() if way too fast!

#define BLUE 2
#define GREEN 1                         // The address of the MAX7221 for the green leds
#define RED 0                           // The address of the MAX7221 for the red leds
int activeMax=GREEN;                // tells which MAX7221 is currently off
unsigned long ISRTime;                   // DEBUG to test how long in ISR
 
LedControl lc=LedControl(12,11,10,3); // pins 10=DataIn, 9=CLK, 8=LOAD + 2 MAX7221s

const int power_pins[2] = {2, 3};
const int signal_pins[5] = {A1, A2, A3, A4, A5};

const int n_cols = 2;
const int n_rows = 5;
 
void setup() {
  lc.setIntensity(GREEN,15);            // 0 = dim, 15 = full brightness
  lc.setIntensity(RED,15);              // red needs less brightness
  lc.setIntensity(BLUE, 10);

  lc.clearDisplay(RED);
  lc.clearDisplay(GREEN);
  lc.clearDisplay(BLUE);
                          // start the timer to toggle shutdown
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(ParseLEDCommand); // register event
  
  for(int i = 0; i != 2; i++){
    pinMode(power_pins[i], OUTPUT);
  }

  for(int i = 0; i != 5; i++){
    pinMode(signal_pins[i], INPUT);
  }
  
  lc.setLed(GREEN, 0, 0, true);
  
  Serial.begin(9600);
  Serial.println("Starting");
  setISRtimer();
  startISR();   
}

String command  = "001102201302";
String command2 = "002101202301";
void loop() {
  /*
   stopISR();
  if(Serial.available()){
   
    command = Serial.readString();
    command.trim();
    Serial.println(command);
    if(command == "rb"){
      CheckBoard();
    }
    else if(command == "wc"){
      // pass
    }

    command = "";
   
  }
  
  startISR();
  */
  
  delay(10);
}

void ParseLEDCommand(int number){
  stopISR();
  Serial.println(number);
  String command = "908";
  startISR();
  if(command == "0"){
    stopISR();
    lc.clearDisplay(RED);
    lc.clearDisplay(GREEN);
    lc.clearDisplay(BLUE);
    startISR();
  }
  //Serial.println(command);
  if(command.length() % 3 != 0){
      return;
  }
  for(int i = 0; i<command.length() - 2; i+=3){
    int color = String(command[i+2]).toInt();
    int row = String(command[i+1]).toInt();
    int col = String(command[i]).toInt();
    //Serial.println(String(color) + " " + String(row) + " " + String(col));

    stopISR();
    lc.setLed(GREEN, row, col, color == GREEN);
    lc.setLed(RED, row, col, color == RED);
    lc.setLed(BLUE, row, col, color == BLUE);
    startISR();
  }
  
}


///////////////////////////Sensor Grid Functions ///////////////////////////
void CheckBoard(){
  for(int i = 0; i != n_cols; i++){
    
    SetColumnPower(i);
    for(int i = 0; i != n_rows; i++){
      Serial.print(String(analogRead(signal_pins[i]) < 40) + " ");
      
    }
    SetColumnPower(-1); 
  }
}

void SetColumnPower(int col){
  for(int i = 0; i != n_cols; i++){
    digitalWrite(power_pins[i], i == col ? HIGH : LOW);
  }
}
 
/////////////////////////////ISR Timer Functions ///////////////////////////
ISR(TIMER1_COMPA_vect) {  //This ISR toggles shutdown between the 2MAX7221's
  if(activeMax==RED){
    lc.shutdown(RED,true);  // The order here is critical - Shutdown first!
    lc.shutdown(GREEN, false);
    activeMax=GREEN;
  }
  else if(activeMax==GREEN){
    lc.shutdown(GREEN,true);  // The order here is critical - Shutdown first!
    lc.shutdown(BLUE, false);
    activeMax=BLUE;
  } else {
    lc.shutdown(BLUE,true);   // . . . Then restart the other.
    lc.shutdown(RED, false);
    activeMax=RED;
  }
}  
 
void setISRtimer(){  // setup ISR timer controling toggleing
  cli();//stop interrupts

  //set timer4 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 110;// = (16*10^6) / (26*256) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}
 
void startISR(){  // Starts the ISR
  TCNT1 = 0;                            // clear counter (needed here also)
  TIMSK1|=(1<<OCIE1A);                  // set interrupts=enabled (calls ISR(TIMER2_COMPA_vect)
}
 
void stopISR(){    // Stops the ISR
  TIMSK1&=~(1<<OCIE1A);                  // disable interrupts
}
 
//////////////////////// simple LED display routine /////////////////////////////
void SkipRows() { // 1st pass alternates green & red, 2nd adds green to red making orange
  stopISR();
  byte greenOn = true;                  // flag for lighting green for orange
  for(int row=0;row<8;row++) {
    for(int col=0;col<8;col++) {
      if (greenOn == true) lc.setLed(GREEN,row,col,true);
      else lc.setLed(RED,row,col,true);
      greenOn = !greenOn;
    }
  }
  delay(500);                           // only so you can see the first pass
  greenOn = !greenOn;
  for(int row=0;row<8;row++) {
    for(int col=0;col<8;col++) {
      delay(4);                         // only so you can see the update
      if (greenOn == true) SetLed(GREEN,row,col,false);
      greenOn = !greenOn;
    }
  }
   startISR();
}
 
/////////   Wrappers for LedControl functions . . . //////////
void SetLed(byte Color, byte Row,byte Col, byte State){
  stopISR();            // disable interrupts - stop toggling shutdown when updating
  lc.setLed(Color,Row,Col,State);
  startISR();           // enable interrupts again
}
 
void SetRow(byte Color, byte Row, byte State){
  stopISR();            // disable interrupts - stop toggling shutdown when updating
  lc.setRow(Color,Row,State);
  startISR();           // enable interrupts again
}
 
void SetColumn(byte Color, byte Col, byte State){
  stopISR();            // disable interrupts - stop toggling shutdown when updating
  lc.setColumn(Color,Col,State);
  startISR();           // enable interrupts again
}
 
void ClearMatrix(){
  stopISR();            // disable interrupts - stop toggling shutdown when updating
  lc.clearDisplay(GREEN);
  lc.clearDisplay(RED);
  startISR();           // enable interrupts again
}

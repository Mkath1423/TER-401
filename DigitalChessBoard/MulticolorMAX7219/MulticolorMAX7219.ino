/* RG Matrix Example   v.2 8/1/08  BroHogan
 * Demos 2 color 8x8 matrix driven by 2 MAX7821's
 */
#include "Arduino.h"                   // needed to compile with Rel. 0013 WHY?!
#include "LedControl.h"                 // to drive the matrix
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
  setISRtimer();                        // setup the timer
                          // start the timer to toggle shutdown
  
  
  for(int i = 0; i != 2; i++){
    pinMode(power_pins[i], OUTPUT);
  }

  for(int i = 0; i != 5; i++){
    pinMode(signal_pins[i], INPUT);
  }

  Serial.begin(9600);
  Serial.println("Starting");
  startISR();   
}

String command = "";
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
   stopISR();
  
   for(int row=0; row != 8;row++) {
   for(int i =0; i != 8; i += 2){
    if(row % 2 ==0){
      SetLed(GREEN, row, i, true);
      SetLed(BLUE, row, i + 1, true);
    }
    else{
      SetLed(BLUE, row, i, true);
      SetLed(GREEN, row, i + 1, true);
    }
   }
  }
  
  startISR(); 
  
  delay(1000);
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
ISR(TIMER2_COMPA_vect) {  //This ISR toggles shutdown between the 2MAX7221's
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
  TCCR2A = 0x02;                        // WGM22=0 + WGM21=1 + WGM20=0 = Mode2 (CTC)
  TCCR2B = 0x05;                // CS22=1 + CS21=0 + CS20=1 = /128 prescaler (125kHz)
  TCNT2 = 0;                            // clear counter
  OCR2A = ISR_FREQ;                     // set TOP (divisor) - see #define
}
 
void startISR(){  // Starts the ISR
  TCNT2 = 0;                            // clear counter (needed here also)
  TIMSK2|=(1<<OCIE2A);                  // set interrupts=enabled (calls ISR(TIMER2_COMPA_vect)
}
 
void stopISR(){    // Stops the ISR
  TIMSK2&=~(1<<OCIE2A);                  // disable interrupts
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

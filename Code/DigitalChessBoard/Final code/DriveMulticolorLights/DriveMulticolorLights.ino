#include "Arduino.h"  
#include "LedControl.h" 
#include "Wire.h"
#define ISR_FREQ 190     //190=650Hz    // Sets the speed of the ISR - LOWER IS FASTER
// prescaler is /128 -  125,000/ISR_FREQ+1 (i.e 249=500Hz, 190=650Hz)
// Tweaked depending on the overhead in the ISR & and other factors in the sketch
// Display will be slow if too FAST. Sketch will hang on delay() if way too fast!
// * from tutorial *

#define BLUE 2
#define GREEN 1    
#define RED 0           

// the Max7219 not in shutdown
int activeMax=GREEN;  

LedControl lc=LedControl(12,11,10,3); // pins 10=DataIn, 9=CLK, 8=LOAD + 2 MAX7221s

 
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
  
  lc.setLed(GREEN, 0, 0, true);
  
  Serial.begin(9600);
  Serial.println("Starting");
  
  setISRtimer();
  startISR();   
}

// do nothing in loop
// everything is done in the ISR
void loop() {
  delay(10);
}

void ParseLEDCommand(int amount){
  // dont interupt when reading incomming data
  stopISR();

  // Read all the incoming data
  String command = "";
  while(0 < Wire.available())       // loop through all but the last
  {
    command += char(Wire.read());         // receive byte as a character
  }
  // restart the interrupts
  startISR();

  // clear the display if the data is "0"
  if(command == "0"){
    stopISR();
    lc.clearDisplay(RED);
    lc.clearDisplay(GREEN);
    lc.clearDisplay(BLUE);
    startISR();
  }
  
  // If the data is not in pairs of 3 then throw it out
  if(command.length() % 3 != 0){
      return;
  }
  // for every color-row-col triple in the data
  for(int i = 0; i<command.length() - 2; i+=3){
    int color = String(command[i+2]).toInt();
    int row = String(command[i+1]).toInt();
    int col = String(command[i]).toInt();

    // set the color at that position
    stopISR();
    lc.setLed(GREEN, row, col, color == GREEN);
    lc.setLed(RED, row, col, color == RED);
    lc.setLed(BLUE, row, col, color == BLUE);
    startISR();
  }
  
}

/////////////////////////////ISR Timer Functions ///////////////////////////
ISR(TIMER1_COMPA_vect) {
  // whenever this function is called switch the Max7219 that is active
  if(activeMax==RED){
    lc.shutdown(RED,true);  
    lc.shutdown(GREEN, false);
    activeMax=GREEN;
  }
  else if(activeMax==GREEN){
    lc.shutdown(GREEN,true);  
    lc.shutdown(BLUE, false);
    activeMax=BLUE;
  } else {
    lc.shutdown(BLUE,true); 
    lc.shutdown(RED, false);
    activeMax=RED;
  }
}  
 
void setISRtimer(){  // setup ISR timer controling toggleing
  //stop interrupts
  cli();
  
  // clear timer1
  TCCR1A = 0;
  TCCR1B = 0;
  
  //initialize counter value to 0
  TCNT1  = 0;
  
  // set the compare match register
  OCR1A = 110;
  
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  
  // Set the prescaler
  TCCR1B |= (1 << CS12);  
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  //allow interrupts
  sei();
}
 
void startISR(){  // Starts the ISR
  TCNT1 = 0;                          
  TIMSK1|=(1<<OCIE1A);
}
 
void stopISR(){  
  TIMSK1&=~(1<<OCIE1A);  
}
 
// Wrappers for LedControl functions 
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

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  Serial.println("I work");
  // put your setup code here, to run once:
  setISRtimer();
  stopISR();
}
boolean toggle4 = LOW;
void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  Serial.println("starting");
  startISR();
  delay(1000);
  Serial.println("stopping");
  stopISR();
}

ISR(TIMER1_COMPA_vect) {  //This ISR toggles shutdown between the 2MAX7221's
  digitalWrite(13,toggle4);
  toggle4 = !toggle4;
}  
 
void setISRtimer(){  // setup ISR timer controling toggleing
  cli();//stop interrupts

  //set timer4 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 100/1;// = (16*10^6) / (300*1024) - 1 (must be <65536)
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
  digitalWrite(13,LOW);
}

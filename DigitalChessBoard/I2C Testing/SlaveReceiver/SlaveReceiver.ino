/*I2C Slave receiver code*/
/*List of the lubraries included in the project*************************************************/
#include <Wire.h>                   //Arduino library that enables I2C functionality  
/*Setup loop************************************************************************************/
void setup()
{
  Serial.begin(9600);               // start serial for output
  Serial.println("Slave started");
  
  Wire.begin(4);                    // join I2C bus with address #4
  Wire.onReceive(receiveEvent);     // register event, that will activate every time that new data is recivied via I2C
 
}
/*Main loop**************************************************************************************/
void loop()
{ 
  delay(100);
}
/*function that is called and executes whenever data is received from master**********************/
void receiveEvent(int howMany)      // this function is registered as an event, see setup() for more information how is this executed
{
  Serial.println("\nreceived event");
  String data = "";
  while(1 < Wire.available())       // loop through all but the last
  {
    data += char(Wire.read());           // receive byte as a character
  }
  int x = Wire.read();              // receive byte as an integer
  Serial.println(data);                // print the integer
}

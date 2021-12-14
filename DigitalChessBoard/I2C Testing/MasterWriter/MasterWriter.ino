/*I2C Master write code*/
/*List of the lubraries included in the project*************************************************/
#include <Wire.h>                     //Arduino library that enables I2C functionality  
/*List of the variables that will be sent via I2C***********************************************/
                       //defining the variable that will be sent 
/*Setup loop************************************************************************************/
void setup()
{
  Wire.begin();                       // join I2C bus (address optional for master, for now)
  Serial.begin(9600);
}
/*Main loop**************************************************************************************/
void loop()
{
  if(Serial.available()){
    char buf[200] = {};
    Serial.readBytes(buf, 200);
    Serial.println(buf);
    Wire.beginTransmission(4);          // transmit to device #4
    Wire.write(buf);                // sends 6 bytes
    Wire.endTransmission();             // stop transmitting
  
    
    Serial.println("found data");                    // incremanting the x varijable by 1 on every pass
    delay(500);                         // dely before the code is runned again
  }
  
}

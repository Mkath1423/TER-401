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
    delay(100);
    int amount = Serial.available();
    
    char * data;
    data = (char *)malloc(amount+1);
    int i = 0;
    while(Serial.available()){
      data[i] = char(Serial.read());
      i ++;
    }
    data[amount] = '\0';

    
    Serial.write(data);
    
    Wire.beginTransmission(4);          // transmit to device #4
    Wire.write(data);                // sends 6 bytes
    Wire.endTransmission();             // stop transmitting
    
  }
  
}

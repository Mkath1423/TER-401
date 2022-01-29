
const int hallPin = A0;
volatile bool ledState = LOW;

void setup() 
{
  pinMode(hallPin,INPUT);
  Serial.begin(9600);
}

void loop() 
{
  Serial.println(analogRead(hallPin));
}

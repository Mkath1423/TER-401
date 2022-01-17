const int sensor_power = 3;
const int sensor_signal = A0;

const int sensor_threshold = 100;

const int led_red = 5;
const int led_green = 6;
const int led_blue = 7;

int last_time = 0;

void setup() {
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_blue, OUTPUT);

  pinMode(sensor_power, OUTPUT);
  pinMode(sensor_signal, INPUT);

  last_time = millis();

  Serial.begin(9600);
}

int count = 0;

void loop() {
  int start_time = millis();

  digitalWrite(sensor_power, HIGH);
  Serial.println(analogRead(sensor_signal));
  if(analogRead(sensor_signal) < sensor_threshold){
    count += start_time - last_time;
  
    switch((count / 500) % 3){
      Serial.println((count / 500) % 3);
      case 0:
        analogWrite(led_red,   HIGH);
        analogWrite(led_green, LOW);
        analogWrite(led_blue,  LOW);
      case 1:
        analogWrite(led_red,   LOW);
        analogWrite(led_green, HIGH);
        analogWrite(led_blue,  LOW);
      case 2:
        analogWrite(led_red,   LOW);
        analogWrite(led_green, LOW);
        analogWrite(led_blue,  HIGH  );
      default:
        analogWrite(led_red,   LOW);
        analogWrite(led_green, LOW);
        analogWrite(led_blue,  LOW);
    }
  }
  digitalWrite(sensor_power, LOW);
  
  

  last_time = start_time;
  delay(10);
}

void setup() {
  pinMode(A0, INPUT);
  pinMode(12, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  digitalWrite(12, HIGH);
  Serial.println(analogRead(A0));
  digitalWrite(12, LOW);
  delay(250);

}

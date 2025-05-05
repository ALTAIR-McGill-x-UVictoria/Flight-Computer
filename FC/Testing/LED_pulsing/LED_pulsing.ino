void setup() {
  pinMode(24, OUTPUT);

}

void loop() {
  // digitalWrite(24, LOW);
  digitalWrite(24, LOW);
  // delayMicroseconds(10000);
  delay(1000);
  digitalWrite(24, HIGH);
  delay(10);
  // delayMicroseconds(5000);

}

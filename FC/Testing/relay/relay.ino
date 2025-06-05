void setup() {
  // put your setup code here, to run once:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(5, HIGH);
  // digitalWrite(6,HIGH);
  digitalWrite(28, 128);
  Serial.println("high");
  // delayMicroseconds(1000);
  delay(1000);
  digitalWrite(28, 128);
  Serial.println("low");
  delay(1000);
  // Serial.println("energized");
  // delay(500);
  // digitalWrite(22, LOW);
  // Serial.println("de-energized");
  // delay(500);
}

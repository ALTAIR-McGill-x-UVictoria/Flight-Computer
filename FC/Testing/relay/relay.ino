void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(28, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(5, HIGH);
  // digitalWrite(6,HIGH);
  digitalWrite(28, HIGH);
  Serial.println("high");
  // delayMicroseconds(1000);
  delay(10);
  digitalWrite(28, LOW);
  Serial.println("low");
  delay(3000);
  // Serial.println("energized");
  // delay(500);
  // digitalWrite(22, LOW);
  // Serial.println("de-energized");
  // delay(500);
}

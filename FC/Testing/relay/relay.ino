void setup() {
  // put your setup code here, to run once:
  pinMode(22, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(22, HIGH);
  Serial.println("energized");
  delay(500);
  digitalWrite(22, LOW);
  Serial.println("de-energized");
  delay(500);
}

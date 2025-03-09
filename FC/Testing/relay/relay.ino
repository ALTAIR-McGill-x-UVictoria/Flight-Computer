void setup() {
  // put your setup code here, to run once:
  pinMode(29, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(29, HIGH);
  Serial.println("energized");
  delay(5000);
  digitalWrite(29, LOW);
  Serial.println("de-energized");
  delay(5000);
}

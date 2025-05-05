void setup() {
  // put your setup code here, to run once:
  

}

void loop() {
  // put your main code here, to run repeatedly:
  float pd1 = analogRead(26);
  float pd2 = analogRead(27);

  // Serial.print("PD1: "); Serial.print(pd1);
  // Serial.print(", PD2: "); Serial.println(pd2);
  Serial.print(pd1); Serial.print(","); Serial.println(pd2);

  delay(100);
}

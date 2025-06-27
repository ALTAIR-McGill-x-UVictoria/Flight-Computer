void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(29, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(5, HIGH);
  // digitalWrite(6,HIGH);
  digitalWrite(29, HIGH);
  Serial.println("high");
  // delayMicroseconds(1000);
  delay(3000);
  digitalWrite(29, LOW);
  Serial.println("low");
  delay(3000);
  // Serial.println("energized");
  // delay(500);
  // digitalWrite(22, LOW);
  // Serial.println("de-energized");
  // delay(500);
}

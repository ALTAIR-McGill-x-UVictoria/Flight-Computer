#define PWM_pin 3

void setup() {
  pinMode(PWM_pin, OUTPUT);
}

void loop() {
  // D = 20%
  analogWrite(PWM_pin, 255/5);
  delay(1000);
  // D = 10%
  analogWrite(PWM_pin, 255/10);
  delay(1000);
  // D = 6.67%
  analogWrite(PWM_pin, 255/15);
  delay(1000);
  // D = 5%
  analogWrite(PWM_pin, 255/20);
  delay(1000);
  // D = 4%
  analogWrite(PWM_pin, 255/25);
  delay(1000);  
  
}



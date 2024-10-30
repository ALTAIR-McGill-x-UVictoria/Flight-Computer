#define PWM_pin1 14
#define PWM_pin2 2
#define PWM_pin3 15

// #define PWM_pin1 1
// #define PWM_pin2 1
// #define PWM_pin3 1

void setup() {
  pinMode(PWM_pin1, OUTPUT);
  pinMode(PWM_pin2, OUTPUT);
  pinMode(PWM_pin3, OUTPUT);
}

void loop() {
  // D = 20%
  analogWrite(PWM_pin1, 255/5);
  analogWrite(PWM_pin2, 255/5);
  analogWrite(PWM_pin3, 255/5);
  delay(1000);
  // D = 10%
  analogWrite(PWM_pin1, 255/10);
  analogWrite(PWM_pin2, 255/10);
  analogWrite(PWM_pin3, 255/10);
  delay(1000);
  // D = 6.67%
  analogWrite(PWM_pin1, 255/15);
  analogWrite(PWM_pin2, 255/15);
  analogWrite(PWM_pin3, 255/15);
  delay(1000);
}



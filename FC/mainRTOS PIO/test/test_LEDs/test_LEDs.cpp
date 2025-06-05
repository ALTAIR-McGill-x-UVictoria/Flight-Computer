#include <Arduino.h>

#define PIN1 2
#define PIN2 11
#define PIN3 12

// PWM Resolution for Teensy 4.1
#define PWM_RESOLUTION 12       // 12-bit resolution (0-4095)
#define PWM_MAX 4095           // Maximum PWM value at 12-bit

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial connection
  
  Serial.println("PWM LED Test Starting");
  
  // Set PWM resolution to 12-bit for finer control
  analogWriteResolution(PWM_RESOLUTION);
  
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
  pinMode(PIN3, OUTPUT);
}

void loop(){
    Serial.println("Maximum Power");
    
    // Use analogWrite with maximum value for full power
    analogWrite(PIN1, PWM_MAX);
    analogWrite(PIN2, PWM_MAX);
    analogWrite(PIN3, PWM_MAX);

    delay(1000);

    Serial.println("OFF");
    analogWrite(PIN1, 0);
    analogWrite(PIN2, 0);
    analogWrite(PIN3, 0);

    delay(1000);
}
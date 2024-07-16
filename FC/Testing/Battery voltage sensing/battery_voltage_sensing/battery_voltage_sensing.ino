#define ANALOG_IN_PIN 23

// Adjust resistor values 
float R2 = 3485;
float R1 = 10040;

float voltage;

void setup() {
  // put your setup code here, to run once:
  pinMode(ANALOG_IN_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  voltage = average_battery_voltage();
  Serial.print(voltage);
  Serial.print(" V");
  Serial.println();
  delay(50);
}

float battery_voltage() {
  int adcValue = analogRead(ANALOG_IN_PIN);
  float resistorRatio = R2/(R2+R1);
  float conversionFactor = 3.3/(1024*resistorRatio);
  float voltage = (float)(adcValue*conversionFactor);
  return voltage;
}

float average_battery_voltage(){
  float average = 0;
  for(int i = 0; i < 1000; i++){
    average += battery_voltage();
  }
  average /= 1000;
  return average;
}
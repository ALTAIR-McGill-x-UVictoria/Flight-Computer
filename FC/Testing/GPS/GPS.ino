#include <SoftwareSerial.h>

// The serial connection to the GPS module
// SoftwareSerial ss(4, 3);
#define GPSserial Serial8


void setup(){
  // Serial.begin(9600);
  // ss.begin(9600);
  GPSserial.begin(9600);
}

void loop(){
  while (GPSserial.available() > 0){
    // get the byte data from the GPS
    byte gpsData = GPSserial.read();
    Serial.write(gpsData);
  }
}
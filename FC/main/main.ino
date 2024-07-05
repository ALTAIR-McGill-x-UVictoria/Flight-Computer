//Flight Computer Main

// #include <RadioLib.h>
#include <ArduinoQueue.h>
#include "Waveshare_10Dof-D.h"
#include "RadioLogic.h"

#define TEENSYDUINO

RadioLogic radio;

// RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RH_RF95 rf95;

void setup() {

  // radio = new RadioLogic();
  // rf95 = radio.rf95
  
  Serial.begin(115200);
  Serial.println("Initializing");
  radio.initializeRadio();
  

  Serial.println("Running main loop");


}

void loop() {
  Serial.println(radio.rf95.getDeviceVersion());//debugging tool
  // put your main code here, to run repeatedly:
  
  radio.radioTx();
  

}

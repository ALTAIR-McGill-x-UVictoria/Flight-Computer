//Flight Computer Main

// #include <RadioLib.h>
#include <ArduinoQueue.h>
#include "Waveshare_10Dof-D.h"
#include "radioLogic.h"

radioLogic radio;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing");
  radio = new radioLogic();
  

  Serial.println("Running main loop");
}

void loop() {
  // put your main code here, to run repeatedly:
  radio.txLoop();

}

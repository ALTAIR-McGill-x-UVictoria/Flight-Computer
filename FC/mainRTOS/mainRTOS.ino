//Flight Computer Main

#include <ArduinoQueue.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <SD.h>
#include <Bonezegei_DRV8825.h>
#include <TeensyThreads.h>
#include "RadioTXHandler.h"
#include "utils.h"



//Callsign
#define CALLSIGN "VA2ETD"


// Singleton instances

// Radio
Radio radio;


void setup(){
  radio.setup();
  sensorSetup();
  SDSetup();

  // Create a thread for the DAQ
  threads.addThread(DAQacquire, 0);
}

void loop() {
    
    updateRadioPacket(radio.lastRSSI, radio.lastSNR);
    char packet[150];
    formRadioPacket(packet, sizeof(packet));
    radio.FCradioHandler(packet);
    delay(500);
}





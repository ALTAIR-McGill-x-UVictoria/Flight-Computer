//Flight Computer Main

#include <ArduinoQueue.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <SD.h>
#include <TeensyThreads.h>
#include "RadioTXHandler.h"
#include "utils.h"

//Callsign
#define CALLSIGN "VA2ETD"

// Macros
#define GPS_SERIAL Serial8
#define GPS_BAUD 9600
#define BUZZER_PIN 22

// Singleton instances

// Radio
Radio radio;

void setup() {
    radio.setup();
    sensorSetup();
    SDSetup();
    
    GPS_SERIAL.begin(9600);  // Begin GPS serial communication
    
    // Create threads
    threads.addThread(DAQacquire, 0);
    threads.addThread(GPSacquire, 1);
}

void loop() {
    
    updateRadioPacket(radio.lastRSSI, radio.lastSNR);
    char packet[150];
    formRadioPacket(packet, sizeof(packet));
    radio.FCradioHandler(packet);
    delay(500);

}





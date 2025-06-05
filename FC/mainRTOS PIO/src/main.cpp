#include <Arduino.h>
//Flight Computer Main

#include <ArduinoQueue.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <SD.h>
#include <TeensyThreads.h>
#include "RadioTXHandler.h"
#include "utils.h"
#include <Arduino.h>
// #include <common/mavlink.h>
// #include <Streaming.h>

// #include "HighSpeedLogger.h"
// #include "linker.h"
// #include "autopilot_interface.h"

//Callsign
#define CALLSIGN "VA2ETD"

// Macros
#define GPS_SERIAL Serial8
#define GPS_BAUD 9600
#define BUZZER_PIN 22

#define USE_ALT_PACKET 1

// Singleton instances

// Radio
Radio radio;
char packet[150];

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    radio.setup();
    sensorSetup();
    SDSetup();

    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    // GPS setup
    GPS_SERIAL.begin(GPS_BAUD);

    // Pixhawk setup
    MAVsetup();
    
    // Initialize GPS data structure
    currentGPSData.valid = false;
    currentGPSData.satellites = 0;
    
    // Create threads
    threads.addThread(DAQacquire, 0);
    threads.addThread(GPSacquire, 1);
    threads.addThread(MAVLinkAcquire, 2);
}

void loop() {
    
    if (USE_ALT_PACKET) {
        updateAltRadioPacket(radio.lastRSSI, radio.lastSNR);
        formAltRadioPacket(packet, sizeof(packet));
        
    } else {
        updateRadioPacket(radio.lastRSSI, radio.lastSNR);
        formRadioPacket(packet, sizeof(packet));
        
    }
    // updateRadioPacket(radio.lastRSSI, radio.lastSNR);
    // char packet[150];
    // formRadioPacket(packet, sizeof(packet));
    
    radio.FCradioHandler(packet);


    delay(250);


}
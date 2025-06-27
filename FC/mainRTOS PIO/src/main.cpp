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
#include "notes.h"
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
// #define BUZZER_PIN 22

#define USE_ALT_PACKET 1
#define DEBUG_PIXHAWK 0

// Singleton instances

// Radio
Radio radio;
char packet[150];

// Functionality status indicators
int radioStatus = 0;
int sensorStatus = 0;
int SDStatus = 0;


void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    // while (!Serial) delay(10);
    if (!Serial) delay(1000);
    
    Serial.println("Flight Computer Starting...");

    // threads.addThread(playWaitingJingle, 5);
    playWaitingJingle();

    // int radioStatus = radio.setup();
    // radioStatus = radio.setup();
    radioStatus = radio.setup();
    sensorStatus = sensorSetup();
    SDStatus = SDSetup();
    
    // GPS setup
    GPS_SERIAL.begin(GPS_BAUD);

    // Pixhawk setup
    MAVsetup();
    
    // Initialize GPS data structure
    currentGPSData.valid = false;
    currentGPSData.satellites = 0;
    
    // Create threads

    threads.addThread(DAQacquire, 0);

    if (sensorStatus >= 1) {
        threads.addThread(GPSacquire, 1);
    } else {
        Serial.println("Sensor setup failed, skipping IMU calibration");
    }
    
    threads.addThread(MAVLinkAcquire, 2);


    if (radioStatus >= 1) {
        Serial.println("Radio setup successful");
    } else {
        playErrorJingle();
        Serial.print("Radio setup failed with error: ");
        Serial.println(radioStatus);
    }

    if (SDStatus >= 1) {
        Serial.println("SD card setup successful");
    } else {
        Serial.print("SD card setup failed with error: ");
        Serial.println(SDStatus);
    }

    if (sensorStatus >= 1) {
        Serial.println("Sensor setup successful");
    } else {
        Serial.print("Sensor setup failed with error: ");
        Serial.println(sensorStatus);
    }


    Serial.println("Flight Computer Setup Complete");
    // threads.kill(5); // Stop the waiting jingle thread
    playStartupJingle();
    
    
}

void loop() {
    char packet[150];
    
    if (radioStatus >= 1) {
        if (USE_ALT_PACKET) {
            updateAltRadioPacket(radio.lastRSSI, radio.lastSNR);
            formAltRadioPacket(packet, sizeof(packet));
        } else {
            updateRadioPacket(radio.lastRSSI, radio.lastSNR);
            formRadioPacket(packet, sizeof(packet));
        }
        
        radio.FCradioHandler(packet);
    }

    // Serial.println(packet);

    #if DEBUG_PIXHAWK
        debugPixhawkData(message);
    #endif

    delay(500);
}

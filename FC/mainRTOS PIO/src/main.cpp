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

#define LED_ENABLE 0

//Callsign
#define CALLSIGN "VA2ETD"

// Termination parameters
#define ENABLE_TERMINATION 1 // Enable termination functionality
#define TARGET_PRESSURE 26.0f // Target pressure in hPa
#define TARGET_ALTITUDE 100.0f // Target altitude in meters

// Macros
#define GPS_SERIAL Serial8
#define GPS_BAUD 9600
// #define BUZZER_PIN 22

#define USE_ALT_PACKET 1
#define DEBUG_PIXHAWK 1


// Singleton instances

// Radio
Radio radio;
char packet[150];

// Functionality status indicators
int radioStatus = 0;
int sensorStatus = 0;
int SDStatus = 0;
// int actuatorEnergized = 0;

void terminationHandler();
// void LEDHandler();
// void trackingLED();
// void sourceLED();
// void trackingLEDCached(uint64_t gps_time_usec);
// void sourceLEDCached(uint64_t gps_time_usec);

void mavlinkThread();


void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    // while (!Serial) delay(10);
    if (!Serial) delay(1000);
    
    Serial.println("Flight Computer Starting...");

    // threads.addThread(playWaitingJingle, 5);
    playWaitingJingle();

    pinMode(TERMINATION_PIN, OUTPUT);
    digitalWrite(TERMINATION_PIN, HIGH);

    //Setup LED pins
    LED_Setup();

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

    // threads.addThread(DAQacquire, 0);

    // if (sensorStatus >= 1) {
    //     threads.addThread(GPSacquire, 1);
    // } else {
    //     Serial.println("Sensor setup failed, skipping IMU calibration");
    // }
    
    threads.addThread(mavlinkUpdateThread, 0);


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

    // #if LED_ENABLE
    //     threads.addThread(LEDHandler, 3);
    // #endif 

    Serial.println("Flight Computer Setup Complete");
    // threads.kill(5); // Stop the waiting jingle thread
    playStartupJingle();
    
    
}

void loop() {
    static unsigned long lastRadioTime = 0;
    static unsigned long lastDebugTime = 0;
    
    // Reduce radio transmission frequency to prevent blocking
    if (millis() - lastRadioTime > 1000) { // Every 1 second instead of 500ms
        char packet[250];
        
        if (radioStatus >= 1) {
            if (USE_ALT_PACKET) {
                updateAltRadioPacket(radio.lastRSSI, radio.lastSNR);
                formAltRadioPacket(packet, sizeof(packet));
                // Serial.println(packet);
            } else {
                updateRadioPacket(radio.lastRSSI, radio.lastSNR);
                formRadioPacket(packet, sizeof(packet));
            }
            
            radio.FCradioHandler(packet);
        }
        lastRadioTime = millis();
    }

    #if DEBUG_PIXHAWK
        // Debug less frequently
        if (millis() - lastDebugTime > 5000) { // Every 5 seconds
            debugPixhawkData(message);
            lastDebugTime = millis();
        }
    #endif

    #if ENABLE_TERMINATION
        terminationHandler();
    #endif 

    MAVLinkAcquire();

    #if LED_ENABLE
        LEDHandler();
    #endif

    photodiodeAcquire();
    batteryAcquire();

    // delay(1);

    
}

void terminationHandler(){
    //Once the payload reaches a target barometer altitude,
    // terminate by actuating linear actuator
    if (message.abs_pressure < TARGET_PRESSURE && message.abs_pressure > 5.0){ // 90 minutes
        digitalWrite(TERMINATION_PIN, LOW);
    }

    if (millis() > 5400000){
        digitalWrite(TERMINATION_PIN, LOW);
    }

}
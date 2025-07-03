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

#define LED_ENABLE 1
#define SOURCE_LED_PIN 6
#define TRACKING_LED_GREEN_PIN 4
#define TRACKING_LED_RED_PIN 5

#define LED_TRACKING_INTERVAL 5000 //ms
#define LED_SOURCE_INTERVAL 20000 //ms


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
void LEDHandler();
void trackingLED();
void sourceLED();
void trackingLEDCached(uint64_t gps_time_usec);
void sourceLEDCached(uint64_t gps_time_usec);

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
    pinMode(SOURCE_LED_PIN, OUTPUT);
    pinMode(TRACKING_LED_GREEN_PIN, OUTPUT);
    pinMode(TRACKING_LED_RED_PIN, OUTPUT);

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

void LEDHandler(){
    static uint64_t cached_gps_time = 0;
    static uint32_t cached_boot_time = 0;
    static unsigned long last_time_update = 0;
    
    // while(1){
        // Update cached time only every 5 seconds to reduce MAVLink calls
        if (millis() - last_time_update > 5) {
            cached_gps_time = message.unix_time_usec;  // Get GPS time in microseconds
            last_time_update = millis();
        }
        
        // Use cached time for LED control
        sourceLEDCached(cached_gps_time);
        trackingLEDCached(cached_gps_time);

        // Serial.println(cached_gps_time);
        
        // delay(100);
        // threads.yield();
    // }
}

void sourceLEDCached(uint64_t gps_time_usec){
    static uint32_t last_gps_seconds = 0;  // Store last known GPS time in seconds
    static unsigned long gps_lost_time = 0;  // When GPS was lost
    static bool had_gps_fix = false;  // Track if we ever had GPS
    
    if (gps_time_usec > 0) {
        // GPS is available - use GPS time
        uint32_t gps_time_seconds = (uint32_t)(gps_time_usec / 1000000);  // Convert microseconds to seconds
        last_gps_seconds = gps_time_seconds;
        had_gps_fix = true;
        
        // Calculate current minute from Unix timestamp
        uint32_t current_minute = (gps_time_seconds / 60) % 60;
        
        Serial.print("GPS Source LED - Current minute: ");
        Serial.print(current_minute);
        Serial.print(" (GPS time: ");
        Serial.print(gps_time_seconds);
        Serial.println(")");

        // Source LED only flashes during EVEN minutes (0, 2, 4, 6, ...)
        if (current_minute % 2 == 0) {
            // Flash at LED_SOURCE_INTERVAL during even minutes
            // Use the seconds within the current minute for timing
            uint32_t seconds_in_minute = gps_time_seconds % 60;
            uint32_t millis_in_second = (gps_time_usec / 1000) % 1000;
            uint32_t total_millis_in_minute = (seconds_in_minute * 1000) + millis_in_second;
            
            if ((total_millis_in_minute % LED_SOURCE_INTERVAL) < (LED_SOURCE_INTERVAL / 2)) {
                digitalWrite(SOURCE_LED_PIN, HIGH);
            } else {
                digitalWrite(SOURCE_LED_PIN, LOW);
            }
        } else {
            // Off during odd minutes
            digitalWrite(SOURCE_LED_PIN, LOW);
        }
    } else if (had_gps_fix) {
        // GPS fix lost - continue timing using Teensy clock from last known GPS time
        if (gps_lost_time == 0) {
            gps_lost_time = millis();  // Mark when GPS was lost
        }
        
        // Calculate continued time: last GPS time + elapsed Teensy time since loss
        uint32_t elapsed_seconds = (millis() - gps_lost_time) / 1000;
        uint32_t continued_seconds = last_gps_seconds + elapsed_seconds;
        uint32_t current_minute = (continued_seconds / 60) % 60;
        
        Serial.print("GPS Lost - Source LED minute: ");
        Serial.println(current_minute);
        
        // Source LED only flashes during EVEN minutes
        if (current_minute % 2 == 0) {
            uint32_t continued_millis = (continued_seconds * 1000) + ((millis() - gps_lost_time) % 1000);
            uint32_t millis_in_minute = continued_millis % 60000;  // Milliseconds within current minute
            
            if ((millis_in_minute % LED_SOURCE_INTERVAL) < (LED_SOURCE_INTERVAL / 2)) {
                digitalWrite(SOURCE_LED_PIN, HIGH);
            } else {
                digitalWrite(SOURCE_LED_PIN, LOW);
            }
        } else {
            digitalWrite(SOURCE_LED_PIN, LOW);
        }
    } else {
        // Never had GPS fix - use simple Teensy clock timing with minute check
        uint32_t teensy_seconds = millis() / 1000;
        uint32_t current_minute = (teensy_seconds / 60) % 60;
        
        Serial.print("No GPS - Source LED minute: ");
        Serial.println(current_minute);
        
        // Source LED only flashes during EVEN minutes
        if (current_minute % 2 == 0) {
            if ((millis() % LED_SOURCE_INTERVAL) < (LED_SOURCE_INTERVAL / 2)) {
                digitalWrite(SOURCE_LED_PIN, HIGH);
            } else {
                digitalWrite(SOURCE_LED_PIN, LOW);
            }
        } else {
            digitalWrite(SOURCE_LED_PIN, LOW);
        }
    }
    
    // Reset GPS lost time when GPS comes back
    if (gps_time_usec > 0 && gps_lost_time != 0) {
        gps_lost_time = 0;
    }
}

void trackingLEDCached(uint64_t gps_time_usec){
    static uint32_t last_gps_seconds = 0;  // Store last known GPS time in seconds
    static unsigned long gps_lost_time = 0;  // When GPS was lost
    static bool had_gps_fix = false;  // Track if we ever had GPS
    
    if (gps_time_usec > 0) {
        // GPS is available - use GPS time
        uint32_t gps_time_seconds = (uint32_t)(gps_time_usec / 1000000);  // Convert microseconds to seconds
        last_gps_seconds = gps_time_seconds;
        had_gps_fix = true;
        
        // Calculate current minute from Unix timestamp
        uint32_t current_minute = (gps_time_seconds / 60) % 60;
        
        Serial.print("GPS Tracking LED - Current minute: ");
        Serial.print(current_minute);
        Serial.print(" (GPS time: ");
        Serial.print(gps_time_seconds);
        Serial.println(")");
        
        // Tracking LED only flashes during ODD minutes (1, 3, 5, 7, ...)
        if (current_minute % 2 == 1) {
            // Flash at LED_TRACKING_INTERVAL during odd minutes
            // Use the seconds within the current minute for timing
            uint32_t seconds_in_minute = gps_time_seconds % 60;
            uint32_t millis_in_second = (gps_time_usec / 1000) % 1000;
            uint32_t total_millis_in_minute = (seconds_in_minute * 1000) + millis_in_second;
            
            if ((total_millis_in_minute % LED_TRACKING_INTERVAL) < (LED_TRACKING_INTERVAL / 2)) {
                digitalWrite(TRACKING_LED_GREEN_PIN, HIGH);
                digitalWrite(TRACKING_LED_RED_PIN, LOW);
            } else {
                digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
                digitalWrite(TRACKING_LED_RED_PIN, HIGH);
            }
        } else {
            // Off during even minutes
            digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
            digitalWrite(TRACKING_LED_RED_PIN, LOW);
        }
    } else if (had_gps_fix) {
        // GPS fix lost - continue timing using Teensy clock from last known GPS time
        if (gps_lost_time == 0) {
            gps_lost_time = millis();  // Mark when GPS was lost
        }
        
        // Calculate continued time: last GPS time + elapsed Teensy time since loss
        uint32_t elapsed_seconds = (millis() - gps_lost_time) / 1000;
        uint32_t continued_seconds = last_gps_seconds + elapsed_seconds;
        uint32_t current_minute = (continued_seconds / 60) % 60;
        
        Serial.print("GPS Lost - Tracking LED minute: ");
        Serial.println(current_minute);
        
        // Tracking LED only flashes during ODD minutes
        if (current_minute % 2 == 1) {
            uint32_t continued_millis = (continued_seconds * 1000) + ((millis() - gps_lost_time) % 1000);
            uint32_t millis_in_minute = continued_millis % 60000;  // Milliseconds within current minute
            
            if ((millis_in_minute % LED_TRACKING_INTERVAL) < (LED_TRACKING_INTERVAL / 2)) {
                digitalWrite(TRACKING_LED_GREEN_PIN, HIGH);
                digitalWrite(TRACKING_LED_RED_PIN, LOW);
            } else {
                digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
                digitalWrite(TRACKING_LED_RED_PIN, HIGH);
            }
        } else {
            digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
            digitalWrite(TRACKING_LED_RED_PIN, LOW);
        }
    } else {
        // Never had GPS fix - use simple Teensy clock timing with minute check
        uint32_t teensy_seconds = millis() / 1000;
        uint32_t current_minute = (teensy_seconds / 60) % 60;
        
        Serial.print("No GPS - Tracking LED minute: ");
        Serial.println(current_minute);
        
        // Tracking LED only flashes during ODD minutes
        if (current_minute % 2 == 1) {
            if ((millis() % LED_TRACKING_INTERVAL) < (LED_TRACKING_INTERVAL / 2)) {
                digitalWrite(TRACKING_LED_GREEN_PIN, HIGH);
                digitalWrite(TRACKING_LED_RED_PIN, LOW);
            } else {
                digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
                digitalWrite(TRACKING_LED_RED_PIN, HIGH);
            }
        } else {
            digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
            digitalWrite(TRACKING_LED_RED_PIN, LOW);
        }
    }
    
    // Reset GPS lost time when GPS comes back
    if (gps_time_usec > 0 && gps_lost_time != 0) {
        gps_lost_time = 0;
    }
}


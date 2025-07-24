#include <Arduino.h>
#include <ArduinoQueue.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <SD.h>
#include <TeensyThreads.h>
#include "RadioTXHandler.h"
#include "utils.h"
#include "notes.h"
#include "MavlinkDecoder.h"

#define CALLSIGN "VA2ETD"

// Termination parameters
#define ENABLE_TERMINATION 1
#define TARGET_PRESSURE 19300.0f
#define TARGET_ALTITUDE 12000

// Macros
#define GPS_SERIAL Serial8
#define GPS_BAUD 9600
#define USE_ALT_PACKET 1
#define DEBUG_PIXHAWK 0

// LED configuration
#define LED_ENABLE 1
#define LED_AS_THREAD 0
#define SOURCE_LED_PIN 6
#define TRACKING_LED_GREEN_PIN 4
#define TRACKING_LED_RED_PIN 5
#define LED_TRACKING_INTERVAL 400 //ms
#define LED_SOURCE_INTERVAL 20000 //ms

// Global variables
Radio radio;
char packet[150];
int radioStatus = 0;
int sensorStatus = 0;
int SDStatus = 0;

// Function declarations
void terminationHandler();
void LEDHandler();

void setup() {
    Serial.begin(115200);
    if (!Serial) delay(1000);
    
    Serial.println("Flight Computer Starting...");
    playWaitingJingle();

    // Setup pins
    pinMode(TERMINATION_PIN, OUTPUT);
    digitalWrite(TERMINATION_PIN, HIGH);
    pinMode(SOURCE_LED_PIN, OUTPUT);
    pinMode(TRACKING_LED_GREEN_PIN, OUTPUT);
    pinMode(TRACKING_LED_RED_PIN, OUTPUT);

    // Initialize systems
    radioStatus = radio.setup();
    sensorStatus = sensorSetup();
    SDStatus = SDSetup();
    // GPS_SERIAL.begin(GPS_BAUD);
    MAVsetup();
    
    // Initialize GPS data
    currentGPSData.valid = false;
    currentGPSData.satellites = 0;
    
    // Create threads
    threads.addThread(mavlinkUpdateThread, 0);

    #if LED_AS_THREAD
        threads.addThread(LEDHandler);
    #endif

    // Status reporting
    if (radioStatus >= 1) Serial.println("Radio setup successful");
    else { playErrorJingle(); Serial.println("Radio setup failed"); }
    
    if (SDStatus >= 1) Serial.println("SD card setup successful");
    else Serial.println("SD card setup failed");
    
    if (sensorStatus >= 1) Serial.println("Sensor setup successful");
    else Serial.println("Sensor setup failed");

    Serial.println("Flight Computer Setup Complete");
    playStartupJingle();
}

void loop() {
    static unsigned long lastRadioTime = 0;
    static unsigned long lastDebugTime = 0;
    static unsigned long lastSDLogTime = 0;
    
    // Radio transmission every 1 second
    if (millis() - lastRadioTime > 1000) {
        if (radioStatus >= 1) {
            updateAltRadioPacket(radio.lastRSSI, radio.lastSNR);
            formAltRadioPacket(packet, sizeof(packet));
            radio.FCradioHandler(packet);
        }
        lastRadioTime = millis();
    }

    // Debug output every 5 seconds
    #if DEBUG_PIXHAWK
        if (millis() - lastDebugTime > 5000) {
            debugPixhawkData(message);
            lastDebugTime = millis();
        }
    #endif

    // Core functions
    #if ENABLE_TERMINATION
        terminationHandler();
    #endif 
    
    MAVLinkAcquire();
    
    #if LED_ENABLE
    #if LED_AS_THREAD == 0
        LEDHandler();
    #endif
    #endif
    
    photodiodeAcquire();
    batteryAcquire();

    // SD logging every 5 seconds
    if (SDStatus >= 1 && millis() - lastSDLogTime > 500) {
        logFlightData();
        lastSDLogTime = millis();
    }
}

void terminationHandler() {
    // Serial.print("Pressure:");
    // Serial.println(message.abs_pressure);
    if ((message.abs_pressure < TARGET_PRESSURE && message.abs_pressure > 5.0) ||
        (millis() > 5400000)) {
        digitalWrite(TERMINATION_PIN, LOW);
    }
}



#if LED_AS_THREAD
void LEDHandler() {
    while(1){
        uint64_t gps_time = 0;
        uint8_t fix_type = 0;
        
        if (mavlink.getGpsTime(gps_time, fix_type)) {
            // GPS available - calculate current minute
            uint64_t gps_seconds = gps_time / 1000000ULL;  // Convert to seconds
            uint32_t current_minute = (gps_seconds / 60) % 60;  // Minutes within hour (0-59)
            
            // Calculate milliseconds within the current minute (0-59999)
            uint64_t seconds_since_minute_start = gps_seconds % 60;  // Seconds within minute (0-59)
            uint32_t microseconds_in_second = (uint32_t)(gps_time % 1000000ULL);
            uint32_t millis_in_second = microseconds_in_second / 1000;
            uint32_t total_millis_in_minute = (seconds_since_minute_start * 1000) + millis_in_second;
            
            if (current_minute % 2 == 0) {
                // Even minutes (0,2,4,6...): Source LED flashes every 20 seconds, tracking LEDs off
                digitalWrite(SOURCE_LED_PIN, (total_millis_in_minute % LED_SOURCE_INTERVAL) < (LED_SOURCE_INTERVAL / 2));
                digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
                digitalWrite(TRACKING_LED_RED_PIN, LOW);
            } else {
                // Odd minutes (1,3,5,7...): Source LED off, tracking LEDs alternate every 250ms
                digitalWrite(SOURCE_LED_PIN, LOW);
                bool green_on = (total_millis_in_minute % LED_TRACKING_INTERVAL) < (LED_TRACKING_INTERVAL / 2);
                digitalWrite(TRACKING_LED_GREEN_PIN, green_on);
                digitalWrite(TRACKING_LED_RED_PIN, !green_on);
            }
            
            // // Enhanced debug every 5 seconds
            // static unsigned long last_debug = 0;
            // if (millis() - last_debug > 5000) {
            //     Serial.print("GPS LED - UTC time: ");
            //     Serial.print((unsigned long)gps_seconds);
            //     Serial.print("s, Minute: ");
            //     Serial.print(current_minute);
            //     Serial.print(" (");
            //     Serial.print(current_minute % 2 == 0 ? "EVEN-Source" : "ODD-Tracking");
            //     Serial.print("), ms_in_min: ");
            //     Serial.print(total_millis_in_minute);
            //     Serial.print(", fix: ");
            //     Serial.println(fix_type);
            //     last_debug = millis();
            // }
        } else {
            // No GPS: Source off, tracking LEDs alternate continuously
            digitalWrite(SOURCE_LED_PIN, LOW);
            bool green_on = (millis() % LED_TRACKING_INTERVAL) < (LED_TRACKING_INTERVAL / 2);
            digitalWrite(TRACKING_LED_GREEN_PIN, green_on);
            digitalWrite(TRACKING_LED_RED_PIN, !green_on);
            
            // static unsigned long last_debug = 0;
            // if (millis() - last_debug > 10000) {
            //     Serial.println("No GPS - fallback LEDs");
            //     last_debug = millis();
            // }
        }
        threads.yield();
    }

}
#else

void LEDHandler() {
    uint64_t gps_time = 0;
    uint8_t fix_type = 0;
    
    if (1) {
        // GPS available - calculate current minute and second
        uint64_t gps_seconds = message.unix_time_usec / 1000000;  // Convert to seconds
        uint32_t current_minute = (gps_seconds / 60) % 60;  // Minutes within hour (0-59)
        uint32_t current_second = gps_seconds % 60;  // Seconds within minute (0-59)
        
        if (current_minute % 2 == 0) {
            // Odd minutes (1,3,5,7...): Source LED flashes every 20 seconds, tracking LEDs off
            // Source LED: ON for 10s (0-9, 20-29, 40-49), OFF for 10s (10-19, 30-39, 50-59)
            bool source_on = ((current_second % 20) < 10);
            digitalWrite(SOURCE_LED_PIN, source_on);
            digitalWrite(TRACKING_LED_GREEN_PIN, LOW);
            digitalWrite(TRACKING_LED_RED_PIN, LOW);
        } else {
            // Even minutes (0,2,4,6...): Source LED off, tracking LEDs alternate every 1 second
            digitalWrite(SOURCE_LED_PIN, LOW);
            // Green LED on for even seconds (0,2,4,6...), Red LED on for odd seconds (1,3,5,7...)
            bool green_on = (current_second % 2) == 0;
            digitalWrite(TRACKING_LED_GREEN_PIN, green_on);
            digitalWrite(TRACKING_LED_RED_PIN, !green_on);
        }
        
        // Enhanced debug every 5 seconds
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 5000) {
            Serial.print("GPS LED - UTC time: ");
            Serial.print((unsigned long)gps_seconds);
            Serial.print("s, Minute: ");
            Serial.print(current_minute);
            Serial.print(", Second: ");
            Serial.print(current_second);
            Serial.print(" (");
            Serial.print(current_minute % 2 == 1 ? "ODD-Source" : "EVEN-Tracking");
            Serial.println(")");
            last_debug = millis();
        }
    } else {
        // No GPS: Source off, tracking LEDs alternate continuously using Teensy seconds
        digitalWrite(SOURCE_LED_PIN, LOW);
        uint32_t teensy_seconds = millis() / 1000;
        bool green_on = (teensy_seconds % 2) == 0;
        digitalWrite(TRACKING_LED_GREEN_PIN, green_on);
        digitalWrite(TRACKING_LED_RED_PIN, !green_on);
        
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 10000) {
            Serial.println("No GPS - fallback LEDs (1 sec intervals)");
            last_debug = millis();
        }
    }
}
#endif


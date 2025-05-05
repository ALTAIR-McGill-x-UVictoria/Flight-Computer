#include <Arduino.h>
#include <common/mavlink.h>
#include <Streaming.h>
#include <SPI.h>
#include <SD.h>

#define VERSION "0.0.1"

#include "HighSpeedLogger.h"
#include "linker.h"
#include "autopilot_interface.h"

HighSpeedLogger logger(new SDClass(SD));
Linker linker(logger);
Autopilot_Interface pixhawk(&linker);

// For timing the serial output
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 500; // Print every 500ms to avoid flooding

void setup() {
  Serial1.begin(921600);
  SerialUSB.begin(921600);

  delay(1000);

  // linker.logger.begin(SDCARD_SS_PIN);

  SerialUSB << "TIMU - " << VERSION << " initialized" << endl;
}

void loop() {
  // Read messages from Pixhawk
  pixhawk.read_messages();
  
  // Get the latest message data from the autopilot interface
  Mavlink_Messages messages = pixhawk.current_messages;
  
  // Only print data periodically to avoid flooding the serial monitor
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    
    // Print a header for the data block
    SerialUSB << "=== PIXHAWK TELEMETRY ===" << endl;
    
    // System status
    SerialUSB << "Mode: " << messages.heartbeat.custom_mode 
              << " | Status: " << (int)messages.heartbeat.system_status << endl;
    
    // Attitude data (roll, pitch, yaw)
    SerialUSB << "Roll: " << degrees(messages.attitude.roll) << "° "
              << "Pitch: " << degrees(messages.attitude.pitch) << "° "
              << "Yaw: " << degrees(messages.attitude.yaw) << "°" << endl;
    
    // GPS data if available
    SerialUSB << "GPS: Lat " << messages.global_position_int.lat/1E7 
              << ", Lon " << messages.global_position_int.lon/1E7
              << ", Alt " << messages.global_position_int.relative_alt/1000.0 << "m" << endl;
    
    // Battery information
    SerialUSB << "Batt: " << messages.sys_status.voltage_battery/1000.0 << "V, " 
              << (int)messages.sys_status.battery_remaining << "%" << endl;
    
    SerialUSB << "===========================" << endl << endl;
  }
}
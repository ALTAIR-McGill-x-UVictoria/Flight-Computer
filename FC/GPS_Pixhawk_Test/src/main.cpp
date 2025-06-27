#include <Arduino.h>
#include "UbloxGPS.h"
#include "MavlinkDecoder.h"

// Debug configuration
#define DEBUG_SERIAL 0       // Set to 1 to enable raw serial debugging
#define DEBUG_SOURCE 0       // 0 = Ublox Serial, 1 = Pixhawk Serial

// Define GPS on Serial8 as in your original code
#define GPS_SERIAL Serial8
#define GPS_BAUD 9600

// Create GPS object
UbloxGPS gps(GPS_SERIAL);

// Create MAVLink decoder object
MavlinkDecoder mavlink;

// Define Pixhawk serial
#define PIXHAWK_SERIAL Serial2
#define PIXHAWK_BAUD 57600

//Callsign
#define CALLSIGN "VA2ETD"

// Timing control
unsigned long lastComparisonTime = 0;
const unsigned long COMPARISON_INTERVAL = 2000; // Compare every 2 seconds

// Last known values from both sources
struct GPSData {
  double lat;
  double lon;
  double alt;
  uint32_t utcTime;
  uint8_t satellites;
  bool valid;
};

GPSData ubloxData = {0, 0, 0, 0, 0, false};
GPSData pixhawkData = {0, 0, 0, 0, 0, false};

// Buffer for debug output
const int DEBUG_BUFFER_SIZE = 128;
char debugLineBuffer[DEBUG_BUFFER_SIZE];
int debugBufferIndex = 0;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("GPS Comparison Test - Starting");
  
#if DEBUG_SERIAL
  // Debug mode banner
  Serial.println("*** RAW SERIAL DEBUG MODE ENABLED ***");
  if (DEBUG_SOURCE == 0) {
    Serial.println("Monitoring Ublox GPS on Serial8");
    GPS_SERIAL.begin(GPS_BAUD);
  } else {
    Serial.println("Monitoring Pixhawk on Serial2");
    PIXHAWK_SERIAL.begin(PIXHAWK_BAUD);
  }
  Serial.println("Data format: [HEX] ASCII");
  Serial.println("------------------------------");
#else
  // Normal initialization
  // Initialize direct GPS
  gps.begin(GPS_BAUD);
  
  // Initialize MAVLink
  mavlink.begin(PIXHAWK_BAUD);
  
  // Request GPS data stream from Pixhawk
  delay(1000);  // Give time for connection to establish
  mavlink.requestSpecificStreams();
  
  Serial.println("Both GPS systems initialized");
#endif
}

void updateUbloxData() {
  // Update the Ublox direct GPS data
  gps.update();
  
  if (gps.hasValidFix()) {
    ubloxData.lat = gps.getLatitude();
    ubloxData.lon = gps.getLongitude();
    ubloxData.alt = gps.getAltitude();
    ubloxData.utcTime = gps.getUTCTime();
    ubloxData.satellites = gps.getSatellitesVisible();
    ubloxData.valid = true;
  }
}

void updatePixhawkData() {
  // Update MAVLink data
  if (mavlink.update()) {
    int32_t lat = 0, lon = 0, alt = 0;
    uint8_t satellites = 0;
    
    if (mavlink.getGPSInfo(lat, lon, alt, satellites)) {
      // Convert from MAVLink format (degE7) to decimal degrees
      pixhawkData.lat = lat / 1e7;
      pixhawkData.lon = lon / 1e7;
      pixhawkData.alt = alt / 1000.0; // Convert from mm to meters
      
      // Get system time as a proxy for GPS time
      uint64_t unix_time_usec;
      uint32_t boot_time_ms;
      if (mavlink.getSystemTime(unix_time_usec, boot_time_ms)) {
        // Convert microseconds since Unix epoch to seconds
        pixhawkData.utcTime = unix_time_usec / 1000000;
      }
      
      pixhawkData.satellites = satellites;
      pixhawkData.valid = true;
    }
  }
}

void compareGPSData() {
  if (ubloxData.valid && pixhawkData.valid) {
    // Calculate differences
    double latDiff = ubloxData.lat - pixhawkData.lat;
    double lonDiff = ubloxData.lon - pixhawkData.lon;
    double altDiff = ubloxData.alt - pixhawkData.alt;
    int32_t timeDiff = ubloxData.utcTime - pixhawkData.utcTime;
    
    // Convert lat/lon differences to meters (approximate)
    // 1 degree of latitude ≈ 111.32 km
    double latDiffMeters = latDiff * 111320;
    // 1 degree of longitude ≈ 111.32 * cos(latitude) km
    double lonDiffMeters = lonDiff * 111320 * cos(ubloxData.lat * PI / 180);
    
    // Calculate horizontal distance
    double horizDistance = sqrt(latDiffMeters*latDiffMeters + lonDiffMeters*lonDiffMeters);
    
    // Print comparison
    Serial.println("======== GPS COMPARISON ========");
    
    Serial.println("--- Ublox GPS (Serial8) ---");
    Serial.print("Position: "); Serial.print(ubloxData.lat, 6); Serial.print(", "); 
    Serial.print(ubloxData.lon, 6); Serial.print(" Alt: "); Serial.print(ubloxData.alt, 1); 
    Serial.print("m Sats: "); Serial.println(ubloxData.satellites);
    Serial.print("UTC Time: "); Serial.println(ubloxData.utcTime);
    
    Serial.println("--- Pixhawk GPS (MAVLink) ---");
    Serial.print("Position: "); Serial.print(pixhawkData.lat, 6); Serial.print(", "); 
    Serial.print(pixhawkData.lon, 6); Serial.print(" Alt: "); Serial.print(pixhawkData.alt, 1); 
    Serial.print("m Sats: "); Serial.println(pixhawkData.satellites);
    Serial.print("UTC Time: "); Serial.println(pixhawkData.utcTime);
    
    Serial.println("--- Differences ---");
    Serial.print("Horizontal distance: "); Serial.print(horizDistance, 2); Serial.println(" meters");
    Serial.print("Altitude difference: "); Serial.print(altDiff, 2); Serial.println(" meters");
    Serial.print("Time difference: "); Serial.print(timeDiff); Serial.println(" seconds");
    
    Serial.println("==============================");
  } else {
    if (!ubloxData.valid) Serial.println("Waiting for valid fix from Ublox GPS...");
    if (!pixhawkData.valid) Serial.println("Waiting for valid fix from Pixhawk GPS...");
  }
}

#if DEBUG_SERIAL
void processDebugSerialData() {
  HardwareSerial& debugSerial = (DEBUG_SOURCE == 0) ? GPS_SERIAL : PIXHAWK_SERIAL;
  
  while (debugSerial.available() > 0) {
    char c = debugSerial.read();
    
    // Print hex value of each byte
    Serial.print("[");
    if (c < 0x10) Serial.print("0"); // Pad with 0 for single-digit hex values
    Serial.print(c, HEX);
    Serial.print("] ");
    
    // Print ASCII character if printable
    if (c >= 32 && c < 127) {
      Serial.print(c);
    } else if (c == '\n') {
      Serial.println("<LF>"); // Line Feed
    } else if (c == '\r') {
      Serial.println("<CR>"); // Carriage Return
    } else {
      Serial.println(".");    // Non-printable character
    }
    
    // Also collect into a line buffer for NMEA sentences
    if (c == '\n') {
      // End of line - print the complete buffer
      debugLineBuffer[debugBufferIndex] = '\0';
      if (debugBufferIndex > 0) {
        Serial.print("NMEA: ");
        Serial.println(debugLineBuffer);
      }
      debugBufferIndex = 0;
    } else if (c != '\r') {
      // Add to buffer if not carriage return
      if (debugBufferIndex < DEBUG_BUFFER_SIZE - 1) {
        debugLineBuffer[debugBufferIndex++] = c;
      }
    }
  }
}
#endif

void loop() {
#if DEBUG_SERIAL
  // Debug mode - just print raw serial data
  processDebugSerialData();
#else
  // Normal operation mode
  // Update GPS data from both sources
  updateUbloxData();
  updatePixhawkData();
  
  // Compare data at regular intervals
  unsigned long currentMillis = millis();
  if (currentMillis - lastComparisonTime >= COMPARISON_INTERVAL) {
    lastComparisonTime = currentMillis;
    compareGPSData();
  }
#endif
}
#ifndef UTILS_H
#define UTILS_H

#include <TeensyThreads.h>
#include <SD.h>
#include "Waveshare_10Dof-D.h"
#include "MavLinkInterface/autopilot_interface.h"

#define GPS_SERIAL Serial8

// Global IMU data structures
extern IMU_ST_ANGLES_DATA stAngles;
extern IMU_ST_SENSOR_DATA stGyroRawData;
extern IMU_ST_SENSOR_DATA stAccelRawData;
extern IMU_ST_SENSOR_DATA stMagnRawData;
extern Threads::Mutex DAQmutex;

// extern HighSpeedLogger logger(new SDClass(SD));
extern HighSpeedLogger logger(&SD);
extern Linker linker(logger);
extern Autopilot_Interface pixhawk(&linker);

// GPS data structure - moved to top before it's used
struct GPSData {
    float time;      // UTC time (HHMMSS.SS)
    float latitude;  // Decimal degrees
    float longitude; // Decimal degrees
    float speed;     // Speed in km/h
    float course;    // Course over ground in degrees
    float altitude;  // Altitude in meters
    float hdop;      // Horizontal dilution of precision
    int date;        // Date (DDMMYY)
    int satellites;  // Number of satellites in view
    bool valid;      // GPS fix valid
};

// Function declarations
bool SDSetup();
void SDWrite(const String& data);
void sensorSetup();
void calibrateIMU();
void DAQacquire();
void GPSacquire();
void playBuzzer(const String& jingleType);
void updateBuzzer();
void photodiodeAcquire(int *photodiodeValue1, int *photodiodeValue2);
void photodiodeSetup();

// GPS parsing functions
bool parseGPSString(const char* gpsString, GPSData& data);
bool parseGPRMC(const char* rmcString, GPSData& data);
bool parseGPGGA(const char* ggaString, GPSData& data);

// Radio packet functions
void formRadioPacket(char* packet, size_t packet_size);
void updateRadioPacket(int rssi = 0, int snr = 0);

extern String currentFilePath;

extern int32_t s32PressureVal;
extern int32_t s32TemperatureVal;
extern int32_t s32AltitudeVal;

// Mutex for DAQ thread

struct radioPacket {
    // Communication data
    int ack;
    int16_t RSSI;
    int SNR;
    // IMU data
    float fRoll;
    float fPitch;
    float fYaw;
    float Pressure;
    float Temperature;
    float Altitude;
    // System status
    bool SDStatus;
    bool actuatorStatus;
    int photodiodeValue1;
    int photodiodeValue2;
    // GPS data
    float gpsLat;
    float gpsLon;
    float gpsAlt;
    float gpsSpeed;
    float gpsTime;
    bool gpsValid;
    // Navigation data

};

extern radioPacket currentPacket;

// Global GPS data instance
extern GPSData currentGPSData;
extern Threads::Mutex GPSmutex;

/**
 * Thread function to handle MAVLink communication with Pixhawk
 * This function runs continuously and manages message exchange
 * with the autopilot
 */
void MAVlinkHandler() {
    // Initialize Serial2 for Pixhawk communication
    Serial2.begin(921600); // Standard baud rate for Pixhawk telemetry
    delay(100); // Short delay for initialization
    
    // Request data streams we're interested in
    // These are common streams for flight data
    pixhawk.request_data_stream(MAV_DATA_STREAM_RAW_SENSORS, 10, true);       // Raw sensor values
    pixhawk.request_data_stream(MAV_DATA_STREAM_EXTENDED_STATUS, 5, true);    // Battery, GPS, etc.
    pixhawk.request_data_stream(MAV_DATA_STREAM_POSITION, 5, true);           // Position data
    pixhawk.request_data_stream(MAV_DATA_STREAM_EXTRA1, 10, true);            // Attitude data
    pixhawk.request_data_stream(MAV_DATA_STREAM_EXTRA2, 5, true);             // VFR HUD data
    
    // Thread loop - runs continuously
    while (true) {
        // Read and process available MAVLink messages
        pixhawk.read_messages(); 
        
        // Short delay to prevent CPU hogging
        threads.yield();
        delay(10);
    }
}

#endif
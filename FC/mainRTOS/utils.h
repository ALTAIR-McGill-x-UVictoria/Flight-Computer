#ifndef UTILS_H
#define UTILS_H

#include <TeensyThreads.h>
#include <SD.h>
#include "Waveshare_10Dof-D.h"

#define GPS_SERIAL Serial8

// Global IMU data structures
extern IMU_ST_ANGLES_DATA stAngles;
extern IMU_ST_SENSOR_DATA stGyroRawData;
extern IMU_ST_SENSOR_DATA stAccelRawData;
extern IMU_ST_SENSOR_DATA stMagnRawData;
extern Threads::Mutex DAQmutex;

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
void DAQacquire();
void GPSacquire();

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

#endif
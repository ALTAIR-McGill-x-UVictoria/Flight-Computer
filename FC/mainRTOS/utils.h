#ifndef UTILS_H
#define UTILS_H

#include <TeensyThreads.h>
#include <SD.h>
#include "Waveshare_10Dof-D.h"

// Global IMU data structures
extern IMU_ST_ANGLES_DATA stAngles;
extern IMU_ST_SENSOR_DATA stGyroRawData;
extern IMU_ST_SENSOR_DATA stAccelRawData;
extern IMU_ST_SENSOR_DATA stMagnRawData;
extern Threads::Mutex DAQmutex;

// Function declarations
bool SDSetup();
void SDWrite(const char* data);
void sensorSetup();
void DAQacquire();

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
};

extern radioPacket currentPacket;

void formRadioPacket(char* packet, size_t packet_size);

void updateRadioPacket(int rssi = 0, int snr = 0);

// GPS data structure
struct GPSData {
    float time;          // UTC time in HHMMSS.SS format
    bool valid;          // Position validity
    float latitude;      // Latitude in degrees
    float longitude;     // Longitude in degrees
    float speed;        // Speed in knots
    float course;       // Course in degrees
    int date;           // Date in DDMMYY format
};

// Global GPS data instance
extern GPSData currentGPSData;
extern Threads::Mutex GPSmutex;

// GPS functions
void GPSacquire();
bool parseGPSString(const char* gpsString, GPSData& data);
bool parseGPRMC(const char* rmcString, GPSData& data);

#endif
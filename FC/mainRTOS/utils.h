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
    int ack;
    int16_t RSSI;
    int SNR;
    float fRoll;
    float fPitch;
    float fYaw;
    float Pressure;
    float Temperature;
    float Altitude;
    bool SDStatus;
    bool actuatorStatus;
};

extern radioPacket currentPacket;

void formRadioPacket(char* packet, size_t packet_size);

void updateRadioPacket(int rssi = 0, int snr = 0);

#endif
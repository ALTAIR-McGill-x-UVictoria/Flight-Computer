#include "utils.h"
#include <stdio.h>

// Define global variables
IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

String currentFilePath;

Threads::Mutex DAQmutex;

radioPacket currentPacket;

bool SDSetup() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        return false;
    }

    int i = 0;
    while (SD.exists(("datalog_" + String(i) + ".txt").c_str())) {
        i++;
    }
    currentFilePath = "datalog_" + String(i) + ".txt";
    return true;
}

void SDWrite(const String& log) {
    File dataFile = SD.open(currentFilePath.c_str(), FILE_WRITE);

    if (dataFile) {
        dataFile.print(millis());
        dataFile.print(": ");
        dataFile.println(log);
        dataFile.close();
    }
}

void sensorSetup() {
    IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
    imuInit(&enMotionSensorType, &enPressureType);
}

void DAQacquire() {
    while(1) {
        // DAQmutex.lock();

        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
        
        // Release the mutex when done
        // DAQmutex.unlock();
        threads.yield();  // Give other threads a chance to run
    }
}

void formRadioPacket(char* packet, size_t packet_size) {
    DAQmutex.lock();  // Lock while accessing shared data
    snprintf(packet, packet_size,
        "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\0\0\0\0\0",
        currentPacket.ack,
        currentPacket.RSSI,
        currentPacket.SNR,
        currentPacket.fRoll,
        currentPacket.fPitch,
        currentPacket.fYaw,
        currentPacket.Pressure,
        currentPacket.Temperature,
        currentPacket.Altitude,
        currentPacket.SDStatus ? 1 : 0,
        currentPacket.actuatorStatus ? 1 : 0
    );
    DAQmutex.unlock();  // Release the lock
}

void updateRadioPacket(int rssi, int snr) {
    DAQmutex.lock();
    
    // Update packet data
    // currentPacket.ack = ;
    currentPacket.RSSI = rssi;
    currentPacket.SNR = snr;
    
    // Update IMU data
    currentPacket.fRoll = stAngles.fRoll;
    currentPacket.fPitch = stAngles.fPitch;
    currentPacket.fYaw = stAngles.fYaw;
    
    // Update environmental data 
    currentPacket.Pressure = (float)s32PressureVal / 100.0f;
    currentPacket.Temperature = (float)s32TemperatureVal / 100.0f;
    currentPacket.Altitude = (float)s32AltitudeVal / 100.0f;
    
    // Update system status
    currentPacket.SDStatus = SD.exists(currentFilePath.c_str());
    currentPacket.actuatorStatus = false;  // Placeholder
    
    DAQmutex.unlock();
}




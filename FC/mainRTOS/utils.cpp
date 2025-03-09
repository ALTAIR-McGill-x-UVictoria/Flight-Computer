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
GPSData currentGPSData;
Threads::Mutex GPSmutex;

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

void GPSacquire() {
    char gpsString[100];
    int index = 0;
    
    while(1) {
        if (Serial1.available()) {
            char c = Serial1.read();
            
            // Store character
            if (c != '\n' && index < 99) {
                gpsString[index++] = c;
            }
            // End of line detected
            else if (c == '\n') {
                gpsString[index] = '\0';  // Null terminate
                
                GPSData tempData;
                if (parseGPSString(gpsString, tempData)) {
                    GPSmutex.lock();
                    currentGPSData = tempData;
                    GPSmutex.unlock();
                }
                
                // Reset for next line
                index = 0;
            }
        }
        threads.yield();  // Give other threads a chance to run
    }
}

void formRadioPacket(char* packet, size_t packet_size) {
    DAQmutex.lock();  // Lock while accessing shared data
    GPSmutex.lock();  // Lock while accessing GPS data
    
    // Clean temporary buffer first
    memset(packet, 0, packet_size);
    
    // Format packet with exact field widths and no null padding
    int written = snprintf(packet, packet_size,
        "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.6f,%.6f,%.2f,%.2f,%.2f,%d",
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
        currentPacket.actuatorStatus ? 1 : 0,
        currentPacket.gpsLat,
        currentPacket.gpsLon,
        currentPacket.gpsAlt,
        currentPacket.gpsSpeed,
        currentPacket.gpsTime,
        currentPacket.gpsValid ? 1 : 0
    );
    
    // Ensure proper termination
    if (written > 0 && written < packet_size) {
        packet[written] = '\n';  // Add newline
        packet[written + 1] = '\0';  // Ensure null termination
    }
    
    GPSmutex.unlock();
    DAQmutex.unlock();
}

void updateRadioPacket(int rssi, int snr) {
    DAQmutex.lock();
    GPSmutex.lock();
    
    // Communication data
    currentPacket.RSSI = rssi;
    currentPacket.SNR = snr;
    // currentPacket.ack = 0;  // Reset ACK to 0 by default

    // IMU data
    currentPacket.fRoll = stAngles.fRoll;
    currentPacket.fPitch = stAngles.fPitch;
    currentPacket.fYaw = stAngles.fYaw;
    currentPacket.Pressure = s32PressureVal / 100;
    currentPacket.Temperature = s32TemperatureVal / 100;
    currentPacket.Altitude = s32AltitudeVal / 100;
    
    // GPS data
    currentPacket.gpsLat = currentGPSData.latitude;
    currentPacket.gpsLon = currentGPSData.longitude;
    currentPacket.gpsAlt = currentPacket.Altitude;  // Using barometric altitude
    currentPacket.gpsSpeed = currentGPSData.speed;
    currentPacket.gpsTime = currentGPSData.time;
    currentPacket.gpsValid = currentGPSData.valid;
    
    GPSmutex.unlock();
    DAQmutex.unlock();
}

bool parseGPSString(const char* gpsString, GPSData& data) {
    // Check if it's a GPRMC sentence
    if (strncmp(gpsString, "$GPRMC", 6) == 0) {
        return parseGPRMC(gpsString, data);
    }
    return false;
}

bool parseGPRMC(const char* rmcString, GPSData& data) {
    char* token;
    char tempStr[100];
    strcpy(tempStr, rmcString);
    
    // First token is $GPRMC
    token = strtok(tempStr, ",");
    if (!token) return false;
    
    // Time
    token = strtok(NULL, ",");
    if (token) data.time = atof(token);
    
    // Status (A=valid, V=invalid)
    token = strtok(NULL, ",");
    if (token) data.valid = (token[0] == 'A');
    
    // Latitude
    token = strtok(NULL, ",");
    if (token) {
        float deg = atof(token);
        data.latitude = (int)(deg/100) + (deg - ((int)(deg/100) * 100))/60.0;
        
        // N/S indicator
        token = strtok(NULL, ",");
        if (token && token[0] == 'S') data.latitude = -data.latitude;
    }
    
    // Longitude
    token = strtok(NULL, ",");
    if (token) {
        float deg = atof(token);
        data.longitude = (int)(deg/100) + (deg - ((int)(deg/100) * 100))/60.0;
        
        // E/W indicator
        token = strtok(NULL, ",");
        if (token && token[0] == 'W') data.longitude = -data.longitude;
    }
    
    // Speed
    token = strtok(NULL, ",");
    if (token) data.speed = atof(token);
    
    // Course
    token = strtok(NULL, ",");
    if (token) data.course = atof(token);
    
    // Date
    token = strtok(NULL, ",");
    if (token) data.date = atoi(token);
    
    return true;
}

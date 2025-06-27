#include "utils.h"
#include "notes.h"  // Include the new header file with note definitions
#include "config.h"
#include <stdio.h>

#include "MavlinkDecoder.h"
#include "MAVLink.h"
#include "Streaming.h"
#include "RadioTXHandler.h"

// Define global variables
IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

// Photodiode values
int photodiodeValue1 = 0;
int photodiodeValue2 = 0;

// Add offset variables for IMU calibration
float fRollOffset = 0.0f;
float fPitchOffset = 0.0f;
float fYawOffset = 0.0f;

String currentFilePath;

// Mutex
Threads::Mutex DAQmutex;
Threads::Mutex GPSmutex;
Threads::Mutex MAVLinkMutex;

// Packet structure
altRadioPacket currentAltPacket;
radioPacket currentPacket;

// GPS data structure
GPSData currentGPSData;

// Radio radio;

// Create an instance of the MAVLink decoder
MavlinkDecoder mavlink;
MavLinkMessage message;



int SDSetup() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        return 0;
    }

    int i = 0;
    while (SD.exists(("datalog_" + String(i) + ".txt").c_str())) {
        i++;
    }
    currentFilePath = "datalog_" + String(i) + ".txt";
    return 1;
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

void calibrateIMU() {
    // Acquire mutex to prevent data race with DAQacquire thread
    DAQmutex.lock();
    
    // Define number of samples and interval
    const int numSamples = 15;
    const int sampleInterval = 100; // 100ms between samples for ~1 second total
    
    // Variables to accumulate readings
    float rollSum = 0.0f;
    float pitchSum = 0.0f;
    float yawSum = 0.0f;
    
    // Temporary data structures for readings
    IMU_ST_ANGLES_DATA sampleAngles;
    IMU_ST_SENSOR_DATA sampleGyro;
    IMU_ST_SENSOR_DATA sampleAccel;
    IMU_ST_SENSOR_DATA sampleMagn;
    
    Serial.println("Calibrating IMU - hold still...");
    
    // Take multiple readings over ~1 second
    for (int i = 0; i < numSamples; i++) {
        imuDataGet(&sampleAngles, &sampleGyro, &sampleAccel, &sampleMagn);
        
        // Accumulate readings
        rollSum += sampleAngles.fRoll;
        pitchSum += sampleAngles.fPitch;
        yawSum += sampleAngles.fYaw;
        
        // Short delay between readings
        delay(sampleInterval);
    }
    
    // Calculate average offsets
    fRollOffset = rollSum / numSamples;
    fPitchOffset = pitchSum / numSamples;
    fYawOffset = yawSum / numSamples;
    
    // Apply offsets immediately to current angle data
    stAngles.fRoll = 0.0f;
    stAngles.fPitch = 0.0f;
    stAngles.fYaw = 0.0f;
    
    // Update radio packet with zeroed angles
    currentPacket.fRoll = 0.0f;
    currentPacket.fPitch = 0.0f;
    currentPacket.fYaw = 0.0f;
    
    DAQmutex.unlock();
    
    Serial.println("IMU calibration complete with averaged offsets:");
    Serial.print("Roll offset: "); Serial.println(fRollOffset);
    Serial.print("Pitch offset: "); Serial.println(fPitchOffset);
    Serial.print("Yaw offset: "); Serial.println(fYawOffset);
    
    // Log to SD card
    SDWrite("IMU calibrated with averaged offsets: " + 
            String(fRollOffset) + ", " + 
            String(fPitchOffset) + ", " + 
            String(fYawOffset));
            
}

int sensorSetup() {
    IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
    imuInit(&enMotionSensorType, &enPressureType);
    
    // Wait briefly for IMU to stabilize
    delay(100);
    
    // Call the dedicated calibration function
    // calibrateIMU();

    return 1;
}

void DAQacquire() {
    while(1) {
        DAQmutex.lock();

        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        
        // Apply offsets to normalize orientation values
        stAngles.fRoll -= fRollOffset;
        stAngles.fPitch -= fPitchOffset;
        stAngles.fYaw -= fYawOffset;
        
        pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

        // photodiode readout
        photodiodeAcquire(&photodiodeValue1, &photodiodeValue2);
        
        // Release the mutex when done
        DAQmutex.unlock();
        threads.yield();  // Give other threads a chance to run
    }
}

void GPSacquire() {
    static char gpsString[100];
    static int index = 0;
    
    while(1) {
        while (GPS_SERIAL.available()) {
            char c = GPS_SERIAL.read();
            
            // Check for line ending
            if (c == '\n' || c == '\r') {
                if (index > 0) {  // Only process non-empty lines
                    gpsString[index] = '\0';  // Null terminate
                    
                    // Only parse if it's a valid NMEA sentence
                    if (gpsString[0] == '$' && index > 6) {
                        GPSData tempData;
                        if (parseGPSString(gpsString, tempData)) {
                            GPSmutex.lock();
                            currentGPSData = tempData;
                            // Only update valid flag if we got a valid fix
                            if (tempData.valid) {
                                currentGPSData.valid = true;
                            }
                            GPSmutex.unlock();
                            
                            // For debugging
                            // Serial.println(gpsString);
                        }
                    }
                    index = 0;  // Reset for next line
                }
            }
            // Store character if we have room
            else if (index < sizeof(gpsString) - 1) {
                gpsString[index++] = c;
            }
        }
        threads.yield();
    }
}

void formRadioPacket(char* packet, size_t packet_size) {
    DAQmutex.lock();
    GPSmutex.lock();
    
    memset(packet, 0, packet_size);
    
    // Add %.2f for bearing in format string
    int written = snprintf(packet, packet_size,
        "FC:%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.6f,%.6f,%.2f,%.2f,%.2f,%d",
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
        currentPacket.photodiodeValue1,
        currentPacket.photodiodeValue2,
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

// New functions for altRadioPacket
void formAltRadioPacket(char* packet, size_t packet_size) {
    DAQmutex.lock();
    GPSmutex.lock();
    MAVLinkMutex.lock(); // Lock MAVLink data access

    memset(packet, 0, packet_size);

    int written = snprintf(packet, packet_size,
        "%d,%d,%d," // ack, RSSI, SNR
        "%llu,%lu," // FC time: fc_unix_time_usec, fc_boot_time_ms
        // "%.6f,%.6f,%.2f,%.2f,%.2f," // FC GPS: lat1, lon1, alt1, speed1, time1
        "%.6f,%.6f,%.2f,%.2f,%.2f," // Pixhawk GPS: lat2, lon2, alt2, speed2, time2
        "%.2f,%.2f,%.2f," // FC IMU: absPressure1, temperature1, altitude1
        "%.2f,%.2f,%.2f," // Pixhawk IMU: absPressure2, temperature2, diffPressure2
        "%d,%d," // FC Status: SDStatus, actuatorStatus
        "%d,%lu,%lu," // Pixhawk Status: logging_active, write_rate, space_left
        "%llu,%lu," // Pixhawk Time: pix_unix_time_usec, pix_boot_time_ms
        "%.2f,%.2f,%.2f,%lu,%lu,%lu," // Vibration: vibe_x, vibe_y, vibe_z, clip_x, clip_y, clip_z
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f," // Navigation: gpsBearing, gpsBearingMagnetic, gpsBearingTrue, gpsBearingGroundSpeed, gpsBearingGroundSpeedMagnetic, gpsBearingGroundSpeedTrue
        "%d,%d", // Photodiodes: value1, value2
        currentAltPacket.ack,
        currentAltPacket.RSSI,
        currentAltPacket.SNR,
        currentAltPacket.fc_unix_time_usec,
        currentAltPacket.fc_boot_time_ms,
        // currentAltPacket.gpsLat1,
        // currentAltPacket.gpsLon1,
        // currentAltPacket.gpsAlt1,
        // currentAltPacket.gpsSpeed1,
        // currentAltPacket.gpsTime1,
        currentAltPacket.gpsLat2,
        currentAltPacket.gpsLon2,
        currentAltPacket.gpsAlt2,
        currentAltPacket.gpsSpeed2,
        currentAltPacket.gpsTime2,
        currentAltPacket.absPressure1,
        currentAltPacket.temperature1,
        currentAltPacket.altitude1,
        currentAltPacket.absPressure2,
        currentAltPacket.temperature2,
        currentAltPacket.diffPressure2,
        currentAltPacket.SDStatus ? 1 : 0,
        currentAltPacket.actuatorStatus ? 1 : 0,
        currentAltPacket.logging_active ? 1 : 0,
        currentAltPacket.write_rate,
        currentAltPacket.space_left,
        currentAltPacket.pix_unix_time_usec,
        currentAltPacket.pix_boot_time_ms,
        currentAltPacket.vibe_x,
        currentAltPacket.vibe_y,
        currentAltPacket.vibe_z,
        currentAltPacket.clipping_x,
        currentAltPacket.clipping_y,
        currentAltPacket.clipping_z,
        currentAltPacket.gpsBearing,
        currentAltPacket.gpsBearingMagnetic,
        currentAltPacket.gpsBearingTrue,
        currentAltPacket.gpsBearingGroundSpeed,
        currentAltPacket.gpsBearingGroundSpeedMagnetic,
        currentAltPacket.gpsBearingGroundSpeedTrue,
        currentAltPacket.photodiodeValue1,
        currentAltPacket.photodiodeValue2,
        currentAltPacket.FC_battery_voltage,
        currentAltPacket.LED_battery_voltage
    );

    // Ensure proper termination
    if (written > 0 && written < packet_size) {
        packet[written] = '\n';  // Add newline
        packet[written + 1] = '\0';  // Ensure null termination
    } else if (written >= packet_size) {
        // Packet was truncated, ensure null termination at the end of the buffer
        packet[packet_size - 1] = '\0';
        // Optionally log an error or indicate truncation
        // SDWrite("Error: altRadioPacket truncated");
    }

    MAVLinkMutex.unlock();
    GPSmutex.unlock();
    DAQmutex.unlock();
}

// void radioSetup(){
//     radio.setup();
// }

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

    // Photodiode data
    currentPacket.photodiodeValue1 = photodiodeValue1;
    currentPacket.photodiodeValue2 = photodiodeValue2;
    
    // GPS data
    currentPacket.gpsLat = currentGPSData.latitude;
    currentPacket.gpsLon = currentGPSData.longitude;
    currentPacket.gpsAlt = currentPacket.Altitude;  // Using barometric altitude
    currentPacket.gpsSpeed = currentGPSData.speed;
    
    // Convert GPS time from UTC to Eastern Time (UTC-4 for EDT or UTC-5 for EST)
    // Format: HHMMSS.SS
    float utcTime = currentGPSData.time;
    int hours = int(utcTime / 10000);
    int minutes = int((utcTime - hours * 10000) / 100);
    float seconds = utcTime - hours * 10000 - minutes * 100;
    
    // Apply timezone offset (EDT = -4)
    int timeZoneOffset = -4;
    hours = (hours + timeZoneOffset + 24) % 24;
    
    // Re-compose the time with timezone adjustment
    currentPacket.gpsTime = hours * 10000 + minutes * 100 + seconds;
    currentPacket.gpsValid = currentGPSData.valid;
    
    GPSmutex.unlock();
    DAQmutex.unlock();
}

void updateAltRadioPacket(int rssi, int snr) {
    DAQmutex.lock();
    GPSmutex.lock();
    MAVLinkMutex.lock(); // Lock MAVLink data access

    // Communication data
    currentAltPacket.RSSI = rssi;
    currentAltPacket.SNR = snr;
    // currentPacket.ack is typically set by the reply parsing in FCradioHandler

    // Time (FC)
    currentAltPacket.fc_boot_time_ms = millis(); // FC boot time
    currentAltPacket.fc_unix_time_usec = 0; // Placeholder: FC Unix time (requires RTC or GPS sync)

    // GPS data 1 (FC)
    // currentAltPacket.gpsLat1 = currentGPSData.latitude;
    // currentAltPacket.gpsLon1 = currentGPSData.longitude;
    // currentAltPacket.gpsAlt1 = currentGPSData.altitude; // Altitude from GPS
    // currentAltPacket.gpsSpeed1 = currentGPSData.speed;  // Speed in km/h

    // Convert FC GPS time from UTC to local time (e.g., Eastern Time UTC-4/UTC-5)
    float fc_utcTime = currentGPSData.time;
    int fc_hours = int(fc_utcTime / 10000);
    int fc_minutes = int((fc_utcTime - fc_hours * 10000) / 100);
    float fc_seconds = fc_utcTime - fc_hours * 10000 - fc_minutes * 100;
    int timeZoneOffset = -4; // Example: EDT (UTC-4). Adjust as needed.
    fc_hours = (fc_hours + timeZoneOffset + 24) % 24;
    // currentAltPacket.gpsTime1 = fc_hours * 10000 + fc_minutes * 100 + fc_seconds;

    // GPS data 2 (Pixhawk) - from MAVLink message
    currentAltPacket.gpsLat2 = message.lat / 1.0e7f; // Convert from int32 (degrees * 1E7)
    currentAltPacket.gpsLon2 = message.lon / 1.0e7f; // Convert from int32 (degrees * 1E7)
    currentAltPacket.gpsAlt2 = message.alt_vfr;      // Altitude from VFR_HUD (meters MSL)
    currentAltPacket.gpsSpeed2 = message.groundspeed * 3.6f; // Convert m/s to km/h
    currentAltPacket.gpsTime2 = message.unix_time_usec;

    // IMU data 1 (FC)
    currentAltPacket.absPressure1 = s32PressureVal / 100.0f;
    currentAltPacket.temperature1 = s32TemperatureVal / 100.0f;
    // currentAltPacket.altitude1 = s32AltitudeVal / 100.0f; // FC Barometric altitude
    currentAltPacket.altitude1 = -1;

    // IMU data 2 (Pixhawk) - from MAVLink message
    currentAltPacket.absPressure2 = message.abs_pressure; // hPa
    currentAltPacket.temperature2 = message.temperature;  // Celsius
    currentAltPacket.diffPressure2 = message.diff_pressure; // hPa

    // FC System status
    currentAltPacket.SDStatus = SD.begin(BUILTIN_SDCARD); // Check SD status dynamically
    currentAltPacket.actuatorStatus = false; // Placeholder: Update with actual actuator status

    // Pixhawk System status - from MAVLink message
    currentAltPacket.logging_active = message.logging_active;
    currentAltPacket.write_rate = message.write_rate;     // Bytes/s
    currentAltPacket.space_left = message.space_left;     // MiB

    // Pixhawk System time
    currentAltPacket.pix_unix_time_usec = message.unix_time_usec;
    currentAltPacket.pix_boot_time_ms = message.boot_time_ms;

    // Vibration data - from MAVLink message
    currentAltPacket.vibe_x = message.vibe_x;
    currentAltPacket.vibe_y = message.vibe_y;
    currentAltPacket.vibe_z = message.vibe_z;
    currentAltPacket.clipping_x = message.clipping_x;
    currentAltPacket.clipping_y = message.clipping_y;
    currentAltPacket.clipping_z = message.clipping_z;

    // Navigation data
    currentAltPacket.gpsBearing = currentGPSData.course; // FC GPS course (true)
    currentAltPacket.gpsBearingMagnetic = message.heading; // Pixhawk compass heading
    currentAltPacket.gpsBearingTrue = currentGPSData.course; // FC GPS course (true)
    currentAltPacket.gpsBearingGroundSpeed = currentGPSData.speed; // FC ground speed (km/h)
    currentAltPacket.gpsBearingGroundSpeedMagnetic = message.groundspeed * 3.6f; // Pixhawk ground speed (km/h)
    currentAltPacket.gpsBearingGroundSpeedTrue = currentGPSData.speed; // FC ground speed (km/h)

    // Photodiode data
    currentAltPacket.photodiodeValue1 = photodiodeValue1;
    currentAltPacket.photodiodeValue2 = photodiodeValue2;

    currentAltPacket.FC_battery_voltage = 0.0f; // Placeholder: Update with actual FC battery voltage
    currentAltPacket.LED_battery_voltage = 0.0f; // Placeholder:

    MAVLinkMutex.unlock();
    GPSmutex.unlock();
    DAQmutex.unlock();
}

bool parseGPSString(const char* gpsString, GPSData& data) {
    // Check for different NMEA sentences
    if (strncmp(gpsString, "$GPRMC", 6) == 0) {
        return parseGPRMC(gpsString, data);
    } else if (strncmp(gpsString, "$GPGGA", 6) == 0) {
        return parseGPGGA(gpsString, data);
    }
    return false;
}

bool parseGPRMC(const char* rmcString, GPSData& data) {
    char* token;
    char tempStr[100];
    strcpy(tempStr, rmcString);
    
    // $GPRMC,171318.00,A,4530.35425,N,07334.54939,W,5.540,2.87,100325,,,A*7B
    token = strtok(tempStr, ",");  // $GPRMC
    if (!token) return false;
    
    // Time (HHMMSS.SS)
    token = strtok(NULL, ",");
    if (token) data.time = atof(token);
    
    // Status (A=valid, V=invalid)
    token = strtok(NULL, ",");
    if (!token || token[0] != 'A') {
        data.valid = false;
        return false;
    }
    data.valid = true;
    
    // Latitude (DDMM.MMMMM)
    token = strtok(NULL, ",");
    if (token) {
        float raw = atof(token);
        int degrees = (int)(raw / 100);
        float minutes = raw - (degrees * 100);
        data.latitude = degrees + (minutes / 60.0);
        
        // N/S indicator
        token = strtok(NULL, ",");
        if (token && token[0] == 'S') data.latitude = -data.latitude;
    }
    
    // Longitude (DDDMM.MMMMM)
    token = strtok(NULL, ",");
    if (token) {
        float raw = atof(token);
        int degrees = (int)(raw / 100);
        float minutes = raw - (degrees * 100);
        data.longitude = degrees + (minutes / 60.0);
        
        // E/W indicator
        token = strtok(NULL, ",");
        if (token && token[0] == 'W') data.longitude = -data.longitude;
    }
    
    // Speed (knots)
    token = strtok(NULL, ",");
    if (token) {
        data.speed = atof(token) * 1.852; // Convert knots to km/h
    }
    
    // Course/Track (true)
    token = strtok(NULL, ",");
    if (token) data.course = atof(token);
    
    // Date (DDMMYY)
    token = strtok(NULL, ",");
    if (token) data.date = atoi(token);
    
    return true;
}

bool parseGPGGA(const char* ggaString, GPSData& data) {
    char* token;
    char tempStr[100];
    strcpy(tempStr, ggaString);
    
    // $GPGGA,171318.00,4530.35425,N,07334.54939,W,1,04,3.40,29.8,M,-32.6,M,,*5F
    token = strtok(tempStr, ",");  // $GPGGA
    if (!token) return false;
    
    // Time
    token = strtok(NULL, ",");
    
    // Latitude
    token = strtok(NULL, ",");
    if (token) {
        float raw = atof(token);
        int degrees = (int)(raw / 100);
        float minutes = raw - (degrees * 100);
        data.latitude = degrees + (minutes / 60.0);
        
        // N/S indicator
        token = strtok(NULL, ",");
        if (token && token[0] == 'S') data.latitude = -data.latitude;
    }
    
    // Longitude
    token = strtok(NULL, ",");
    if (token) {
        float raw = atof(token);
        int degrees = (int)(raw / 100);
        float minutes = raw - (degrees * 100);
        data.longitude = degrees + (minutes / 60.0);
        
        // E/W indicator
        token = strtok(NULL, ",");
        if (token && token[0] == 'W') data.longitude = -data.longitude;
    }
    
    // Fix quality
    token = strtok(NULL, ",");
    if (!token || token[0] == '0') {
        data.valid = false;
        return false;
    }
    data.valid = true;
    
    // Number of satellites
    token = strtok(NULL, ",");
    if (token) data.satellites = atoi(token);
    
    // HDOP
    token = strtok(NULL, ",");
    if (token) data.hdop = atof(token);
    
    // Altitude
    token = strtok(NULL, ",");
    if (token) data.altitude = atof(token);
    
    return true;
}


void photodiodeSetup() {
    return;
}

void photodiodeAcquire(int *photodiodeValue1, int *photodiodeValue2) {
    *photodiodeValue1 = analogRead(26);
    *photodiodeValue2 = analogRead(27);
}

int MAVsetup(){
    mavlink.begin(921600);
    return 1;
}

void MAVLinkAcquire(){
    // MavLink thread function
    while(1){
        mavlink.update();
        mavlink.getAttitude(message.roll, message.pitch, message.yaw);
        mavlink.getGPSInfo(message.lat, message.lon, message.alt, message.satellites);
        mavlink.getBatteryInfo(message.voltage, message.current, message.remaining);
        mavlink.getVfrHudData(message.airspeed, message.groundspeed, message.heading, message.throttle, message.alt_vfr, message.climb);
        mavlink.getHighResImu(message.acc_x, message.acc_y, message.acc_z, message.xgyro, message.ygyro, message.zgyro, message.xmag, message.ymag, message.zmag, message.abs_pressure, message.diff_pressure, message.temperature);
        mavlink.getSystemTime(message.unix_time_usec, message.boot_time_ms);
        mavlink.getLoggingStats(message.write_rate, message.space_left);
        mavlink.getVibrationData(message.vibe_x, message.vibe_y, message.vibe_z, message.clipping_x, message.clipping_y, message.clipping_z);
    }
}

void debugPixhawkData(MavLinkMessage &message){
    // Lock the mutex before accessing MAVLink data
    MAVLinkMutex.lock();
    
    Serial.println("Pixhawk Data:");
    Serial.print("Roll: "); Serial.println(message.roll);
    Serial.print("Pitch: "); Serial.println(message.pitch);
    Serial.print("Yaw: "); Serial.println(message.yaw);
    Serial.print("Latitude: "); Serial.println(message.lat / 1.0e7f); // Convert from int32 (degrees * 1E7)
    Serial.print("Longitude: "); Serial.println(message.lon / 1.0e7f); // Convert from int32 (degrees * 1E7)
    Serial.print("Altitude: "); Serial.println(message.alt_vfr); // Altitude in meters MSL
    Serial.print("Groundspeed: "); Serial.println(message.groundspeed);
    Serial.print("Heading: "); Serial.println(message.heading);
    Serial.print("Battery Voltage: "); Serial.println(message.voltage);
    Serial.print("Battery Current: "); Serial.println(message.current);
    Serial.print("Remaining Battery: "); Serial.println(message.remaining);
    // Add more fields as needed

    Serial.println("==========================");
    
    // Unlock the mutex when done
    MAVLinkMutex.unlock();
}
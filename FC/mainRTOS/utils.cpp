#include "utils.h"
#include "notes.h"  // Include the new header file with note definitions
#include <stdio.h>

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

Threads::Mutex DAQmutex;
GPSData currentGPSData;
Threads::Mutex GPSmutex;

radioPacket currentPacket;

// Define buzzer pin
#define BUZZER_PIN 22

// Structure to hold tone pattern data
struct ToneStep {
    int frequency;  // frequency in Hz, 0 means silence
    int duration;   // duration in ms
};

// Jingle patterns - define up to 8 steps for each jingle
const ToneStep calibrationJingle[8] = {
    {NOTE_C4, 100}, {0, 50}, {NOTE_E4, 100}, {0, 50}, 
    {NOTE_G4, 100}, {0, 100}, {NOTE_C5, 500}, {0, 0}
};

const ToneStep successJingle[8] = {
    {NOTE_C4, 150}, {0, 50}, {NOTE_E4, 150}, {0, 50}, 
    {NOTE_G4, 300}, {0, 0}, {0, 0}, {0, 0}
};

const ToneStep errorJingle[8] = {
    {NOTE_G4, 300}, {0, 50}, {NOTE_E4, 150}, {0, 50}, 
    {NOTE_C4, 100}, {0, 0}, {0, 0}, {0, 0}
};

const ToneStep warningJingle[8] = {
    {NOTE_A4, 200}, {0, 200}, {NOTE_A4, 200}, {0, 0}, 
    {0, 0}, {0, 0}, {0, 0}, {0, 0}
};

// Global variables for non-blocking buzzer control
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
String currentJingle = "";
int jingleStep = 0;
unsigned long lastBeepTime = 0;

// Global variables for tracking jingle playback state
const ToneStep* currentPattern = nullptr;
int maxSteps = 0;

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

void calibrateIMU() {
    // Acquire mutex to prevent data race with DAQacquire thread
    DAQmutex.lock();
    
    // Play calibration jingle to indicate start
    playBuzzer("calibration");
    
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
            
    // Play success jingle after calibration
    playBuzzer("success");
}

void sensorSetup() {
    IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
    imuInit(&enMotionSensorType, &enPressureType);
    
    // Wait briefly for IMU to stabilize
    delay(100);
    
    // Call the dedicated calibration function
    // calibrateIMU();
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
        "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.6f,%.6f,%.2f,%.2f,%.2f,%d",
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

// Buzzer handler function that plays different jingles
void playBuzzer(const String& jingleType) {
    // If buzzer is already active, don't interrupt current jingle
    if (buzzerActive) return;
    
    currentJingle = jingleType;
    jingleStep = 0;
    buzzerActive = true;
    lastBeepTime = millis();
    
    // Select the appropriate jingle pattern
    if (jingleType == "calibration") {
        currentPattern = calibrationJingle;
        maxSteps = 8;
    }
    else if (jingleType == "success") {
        currentPattern = successJingle;
        maxSteps = 6;
    }
    else if (jingleType == "error") {
        currentPattern = errorJingle;
        maxSteps = 6;
    }
    else if (jingleType == "warning") {
        currentPattern = warningJingle;
        maxSteps = 4;
    }
    else {
        // Default simple beep
        currentPattern = warningJingle;
        maxSteps = 2;
    }
    
    // Start playing the first tone
    if (currentPattern[0].frequency > 0) {
        tone(BUZZER_PIN, currentPattern[0].frequency);
    } else {
        noTone(BUZZER_PIN);
    }
    
    Serial.print("Playing jingle: ");
    Serial.println(jingleType);
}

// Function to update buzzer state (call this in main loop)
void updateBuzzer() {
    if (!buzzerActive || currentPattern == nullptr) return;
    
    unsigned long currentTime = millis();
    
    // Check if it's time to move to the next step
    if (currentTime - lastBeepTime >= currentPattern[jingleStep].duration) {
        // Move to next step
        jingleStep++;
        
        // Check if we've reached the end of the pattern
        if (jingleStep >= maxSteps || currentPattern[jingleStep].duration == 0) {
            noTone(BUZZER_PIN);
            buzzerActive = false;
            return;
        }
        
        // Play the next tone (or silence)
        if (currentPattern[jingleStep].frequency > 0) {
            tone(BUZZER_PIN, currentPattern[jingleStep].frequency);
        } else {
            noTone(BUZZER_PIN);
        }
        
        lastBeepTime = currentTime;
    }
}

void photodiodeSetup() {
    return;
}

void photodiodeAcquire(int *photodiodeValue1, int *photodiodeValue2) {
    *photodiodeValue1 = analogRead(26);
    *photodiodeValue2 = analogRead(27);
}
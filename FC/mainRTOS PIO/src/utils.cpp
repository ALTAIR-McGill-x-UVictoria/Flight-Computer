#include "utils.h"
#include "notes.h"  // Include the new header file with note definitions
#include "config.h"
#include <stdio.h>
#include <SD.h>

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

// Battery voltage reading for 3S1P battery on pin 24
float batteryVoltage = 0.0f;

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



// Add these variables at the top with other globals
static bool sdCardInitialized = false;
static String logFileName = "flight_log.txt";
static unsigned long logCounter = 0;

int SDSetup() {
    Serial.println("Initializing SD card...");
    Serial.print("Using Teensy 4.1 built-in SD card (pin 254)...");
    
    // Add a small delay before initialization
    delay(1000);
    
    // Use pin 254 directly for Teensy 4.1 built-in SD card
    if (SD.begin(254)) {
        Serial.println(" SUCCESS!");
        sdCardInitialized = true;
    } else {
        Serial.println(" FAILED!");
        Serial.println("SD CARD INITIALIZATION FAILED!");
        Serial.println("Troubleshooting steps:");
        Serial.println("1. Ensure SD card is properly inserted");
        Serial.println("2. Check SD card is formatted as FAT32");
        Serial.println("3. Try a different SD card");
        Serial.println("4. Check card is not write-protected");
        Serial.println("5. Try reformatting the card");
        Serial.println();
        Serial.println("Continuing without SD card...");
        
        sdCardInitialized = false;
        return 0; // Return 0 to indicate failure
    }
    
    // Print SD card information
    printSDCardInfo();
    
    // Find next available log file number
    int fileIndex = 0;
    while (SD.exists(("flight_log_" + String(fileIndex) + ".txt").c_str())) {
        fileIndex++;
    }
    logFileName = "flight_log_" + String(fileIndex) + ".txt";
    currentFilePath = logFileName; // Update the global variable
    
    Serial.print("Using log file: ");
    Serial.println(logFileName);
    
    // Create header for new log file
    File dataFile = SD.open(logFileName.c_str(), FILE_WRITE);
    if (dataFile) {
        dataFile.println("Flight Computer Data Log");
        dataFile.println("========================");
        dataFile.print("Start Time: ");
        dataFile.println(millis());
        dataFile.println("GPS_Unix_Time_usec,FC_Boot_Time_ms,GPS_Lat,GPS_Lon,GPS_Alt,Pixhawk_Pressure,Pixhawk_Alt,Teensy_Temp,Pixhawk_Temp,FC_Battery_V,LED_Battery_V,Photodiode1,Photodiode2,LED_Source,LED_Green,LED_Red,Termination,ACK,RSSI,SNR");
        dataFile.close();
        Serial.println("Created new log file with header");
    } else {
        Serial.println("Failed to create log file header");
        return 0;
    }
    
    return 1; // Success
}

void SDWrite(const String& data) {
    // Only attempt to write if SD card was successfully initialized
    if (!sdCardInitialized) {
        return;
    }
    
    File dataFile = SD.open(logFileName.c_str(), FILE_WRITE);
    if (dataFile) {
        logCounter++;
        dataFile.print(millis());
        dataFile.print(",");
        dataFile.println(data);
        dataFile.close();
        
        // Optional: Print to serial for debugging (remove in production)
        // Serial.print("SD Log #");
        // Serial.print(logCounter);
        // Serial.print(": ");
        // Serial.println(data);
    } else {
        Serial.println("Error: Failed to open SD card file for writing");
        // Try to reinitialize SD card with pin 254
        if (SD.begin(254)) {
            Serial.println("SD card reinitialized successfully");
            sdCardInitialized = true;
        } else {
            Serial.println("SD card reinitialize failed");
            sdCardInitialized = false;
        }
    }
}

// Add these new functions to utils.cpp
void printSDCardInfo() {
    Serial.println("\nSD Card Information:");
    Serial.println("--------------------");
    if (sdCardInitialized) {
        Serial.println("SD card initialized successfully");
        Serial.println("Ready for logging operations");
        
        // Try to get card info
        File root = SD.open("/");
        if (root) {
            Serial.println("Root directory accessible");
            root.close();
        }
    } else {
        Serial.println("SD card not initialized");
    }
    Serial.println();
}

void listSDFiles() {
    if (!sdCardInitialized) {
        Serial.println("SD card not available");
        return;
    }
    
    Serial.println("\nFiles on SD card:");
    Serial.println("=================");
    
    File root = SD.open("/");
    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            break; // No more files
        }
        
        Serial.print(entry.name());
        if (entry.isDirectory()) {
            Serial.println("/");
        } else {
            Serial.print("\t\t");
            Serial.print(entry.size(), DEC);
            Serial.println(" bytes");
        }
        entry.close();
    }
    root.close();
    Serial.println();
}

void checkSDCardStatus() {
    Serial.println("\nSD Card Status Check:");
    Serial.println("====================");
    
    Serial.print("Initialized: ");
    Serial.println(sdCardInitialized ? "YES" : "NO");
    
    Serial.print("Current log file: ");
    Serial.println(logFileName);
    
    Serial.print("Log entries written: ");
    Serial.println(logCounter);
    
    if (sdCardInitialized && SD.exists(logFileName.c_str())) {
        File dataFile = SD.open(logFileName.c_str());
        if (dataFile) {
            Serial.print("Log file size: ");
            Serial.print(dataFile.size());
            Serial.println(" bytes");
            dataFile.close();
        }
    }
    
    // Test write capability
    Serial.println("Testing write capability...");
    SDWrite("Test,0,0,0,0,0,0,0,0,0,0"); // Test write
    
    Serial.println("SD Card check complete.");
    Serial.println();
}

// Enhanced logging function for flight data
void logFlightData() {
    if (!sdCardInitialized) {
        return;
    }
    
    DAQmutex.lock();
    GPSmutex.lock();
    
    // Read LED pin states (GPIO status)
    int led_source = digitalRead(6);           // Source LED (pin 6)
    int led_tracking_green = digitalRead(4);   // Green tracking LED (pin 4)
    int led_tracking_red = digitalRead(5);     // Red tracking LED (pin 5)
    
    // Read termination pin state
    int termination_state = digitalRead(29);
    
    // Get Teensy 4.1 internal temperature
    float teensy_temp = tempmonGetTemp();
    
    // SIMPLIFIED: Direct GPS time access without caching
    uint64_t current_gps_time = 0;
    uint8_t fix_type;
    
    // Get current GPS time directly
    if (mavlink.getGpsTime(current_gps_time, fix_type)) {
        // Use fresh GPS time directly
        Serial.print("SD Log: GPS time ");
        Serial.println((unsigned long)(current_gps_time / 1000000));
    } else {
        // Use message time directly
        current_gps_time = message.unix_time_usec;
        Serial.print("SD Log: Message time ");
        Serial.println((unsigned long)(current_gps_time / 1000000));
    }
    
    // FIXED: Proper GPS altitude conversion for logging
    float gps_altitude_meters;
    if (message.alt != 0) {
        gps_altitude_meters = message.alt / 1000.0f; // Convert mm to meters
    } else {
        gps_altitude_meters = message.alt_vfr; // Fallback to VFR altitude
    }
    
    // Create comprehensive flight data string with GPIO status
    String logData = 
        // Time data first - use current GPS time directly
        String(current_gps_time) + "," +                   // GPS UTC time (unix usec)
        String(millis()) + "," +                           // FC boot time (ms)
        
        // GPS data - FIXED: Use properly converted altitude
        String(message.lat / 1.0e7f, 7) + "," +           // GPS latitude
        String(message.lon / 1.0e7f, 7) + "," +           // GPS longitude
        String(gps_altitude_meters, 2) + "," +             // GPS altitude in meters
        
        // Barometric data
        String(message.abs_pressure, 2) + "," +           // Pixhawk barometer pressure
        String(message.pressure_alt, 2) + "," +           // Pixhawk barometer altitude
        
        // Temperature data
        String(teensy_temp, 2) + "," +                    // Teensy 4.1 temperature
        String(message.temperature, 2) + "," +            // Pixhawk temperature
        
        // Power/Battery data
        String(message.voltage, 2) + "," +                // FC battery voltage (from Pixhawk)
        String(getBatteryVoltage(), 2) + "," +            // LED battery voltage
        
        // Sensor data
        String(photodiodeValue1) + "," +                  // Photodiode 1
        String(photodiodeValue2) + "," +                  // Photodiode 2
        
        // GPIO pin states (UPDATED: Added individual pin status)
        String(led_tracking_green) + "," +                // GPIO Pin 4 status (0=LOW, 1=HIGH)
        String(led_tracking_red) + "," +                  // GPIO Pin 5 status (0=LOW, 1=HIGH)
        String(led_source) + "," +                        // GPIO Pin 6 status (0=LOW, 1=HIGH)
        String(termination_state) + "," +                 // Termination pin 29 status
        
        // Communication data
        String(currentAltPacket.ack) + "," +              // Command ACK
        String(currentAltPacket.RSSI) + "," +             // RSSI
        String(currentAltPacket.SNR);                     // SNR
    
    GPSmutex.unlock();
    DAQmutex.unlock();
    
    SDWrite(logData);
}

void calibrateIMU() {
    // Acquire mutex to prevent data race with DAQacquire thread
    DAQmutex.lock();
    
    // Define number of samples and interval
    const int numSamples = 15;
    const int sampleInterval = 100; // 100ms between samples for ~1 second
    
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
        photodiodeAcquire();
        
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

    memset(packet, 0, packet_size);

    // FIXED: Ensure parameter order matches format string exactly
    int written = snprintf(packet, packet_size,
        "%d,%d,%d," // ack, RSSI, SNR
        ",%lu," // FC time: fc_unix_time_usec, fc_boot_time_ms
        "%.6f,%.6f,%.2f,%.2f,%llu," // Pixhawk GPS: lat2, lon2, alt2, speed2, time2
         // FC IMU: absPressure1, temperature1, altitude1
        "%.2f,%.2f,%.2f," // Pixhawk IMU: absPressure2, temperature2, altitude1
        "%.2f,%.2f,%.2f," // Pixhawk Acceleration: acc_x, acc_y, acc_z
        "%d,%d," // FC Status: SDStatus, actuatorStatus
        "%d,%lu,%lu," // Pixhawk Status: logging_active, write_rate, space_left
        // ",," // Pixhawk Time: pix_unix_time_usec, pix_boot_time_ms
        // ",,,,,," // Vibration: vibe_x, vibe_y, vibe_z, clip_x, clip_y, clip_z
        "%.2f," // Navigation: gpsBearing, gpsBearingMagnetic, gpsBearingTrue, gpsBearingGroundSpeed, gpsBearingGroundSpeedMagnetic, gpsBearingGroundSpeedTrue
        "%d,%d," // Photodiodes: value1, value2
        "%.2f,%.2f", // FIXED: Added battery voltages format specifiers
        currentAltPacket.ack,
        currentAltPacket.RSSI,
        currentAltPacket.SNR,
        // currentAltPacket.fc_unix_time_usec,
        currentAltPacket.fc_boot_time_ms,
        currentAltPacket.gpsLat2,
        currentAltPacket.gpsLon2,
        currentAltPacket.gpsAlt2,
        currentAltPacket.gpsSpeed2,
        currentAltPacket.gpsTime2,
        // currentAltPacket.absPressure1,
        // currentAltPacket.temperature1,
        // currentAltPacket.altitude1,
        currentAltPacket.absPressure2,
        currentAltPacket.temperature2,
        currentAltPacket.altitude1,
        currentAltPacket.acc_x,
        currentAltPacket.acc_y,
        currentAltPacket.acc_z,
        currentAltPacket.SDStatus ? 1 : 0,
        currentAltPacket.actuatorStatus ? 1 : 0,
        currentAltPacket.logging_active ? 1 : 0,
        currentAltPacket.write_rate,
        currentAltPacket.space_left,
        // currentAltPacket.pix_unix_time_usec,
        // currentAltPacket.pix_boot_time_ms,
        currentAltPacket.gpsBearing,
        // currentAltPacket.gpsBearingMagnetic,
        // currentAltPacket.gpsBearingTrue,
        // currentAltPacket.gpsBearingGroundSpeed,
        // currentAltPacket.gpsBearingGroundSpeedMagnetic,
        // currentAltPacket.gpsBearingGroundSpeedTrue,
        currentAltPacket.photodiodeValue1,
        currentAltPacket.photodiodeValue2,
        currentAltPacket.FC_battery_voltage,      // FIXED: Added these values
        currentAltPacket.LED_battery_voltage      // FIXED: Added these values
    );

    // Ensure proper termination
    if (written > 0 && written < packet_size) {
        packet[written] = '\n';  // Add newline
        packet[written + 1] = '\0';  // Ensure null termination
    } else if (written >= packet_size) {
        // Packet was truncated, ensure null termination at the end of the buffer
        packet[packet_size - 1] = '\0';
        // Log truncation for debugging
        Serial.println("Warning: altRadioPacket was truncated");
    }

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

    Serial.println(photodiodeValue1);
    Serial.println(photodiodeValue2);
    
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

    // Communication data
    currentAltPacket.RSSI = rssi;
    currentAltPacket.SNR = snr;

    // Time (FC)
    currentAltPacket.fc_boot_time_ms = millis();
    currentAltPacket.fc_unix_time_usec = 0; // Placeholder

    // GPS data 2 (Pixhawk) - from MAVLink message
    currentAltPacket.gpsLat2 = message.lat / 1.0e7f;
    currentAltPacket.gpsLon2 = message.lon / 1.0e7f;
    currentAltPacket.gpsAlt2 = message.alt / 1000.0f;
    currentAltPacket.gpsSpeed2 = message.groundspeed * 3.6f;
    
    currentAltPacket.gpsTime2 = message.unix_time_usec / 1000000;

    // IMU data 1 (FC)
    currentAltPacket.absPressure1 = s32PressureVal / 100.0f;
    currentAltPacket.temperature1 = s32TemperatureVal / 100.0f;
    currentAltPacket.altitude1 = message.pressure_alt;

    // IMU data 2 (Pixhawk) - from MAVLink message
    currentAltPacket.absPressure2 = message.abs_pressure;
    currentAltPacket.temperature2 = message.temperature;
    currentAltPacket.diffPressure2 = message.diff_pressure;

    // Acceleration data - from MAVLink message
    currentAltPacket.acc_x = message.acc_x;
    currentAltPacket.acc_y = message.acc_y;
    currentAltPacket.acc_z = message.acc_z;

    // FC System status
    currentAltPacket.SDStatus = sdCardInitialized;
    currentAltPacket.actuatorStatus = digitalRead(TERMINATION_PIN) == LOW;

    // Pixhawk System status - from MAVLink message
    currentAltPacket.logging_active = message.logging_active;
    currentAltPacket.write_rate = message.write_rate;
    currentAltPacket.space_left = message.space_left;

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
    currentAltPacket.gpsBearing = currentGPSData.course;
    currentAltPacket.gpsBearingMagnetic = message.heading;
    currentAltPacket.gpsBearingTrue = currentGPSData.course;
    currentAltPacket.gpsBearingGroundSpeed = currentGPSData.speed;
    currentAltPacket.gpsBearingGroundSpeedMagnetic = message.groundspeed * 3.6f;
    currentAltPacket.gpsBearingGroundSpeedTrue = currentGPSData.speed;

    // Photodiode data
    currentAltPacket.photodiodeValue1 = photodiodeValue1;
    currentAltPacket.photodiodeValue2 = photodiodeValue2;

    // Battery data
    currentAltPacket.FC_battery_voltage = message.voltage;
    currentAltPacket.LED_battery_voltage = getBatteryVoltage();

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

void photodiodeAcquire() {
    photodiodeValue1 = analogRead(25);
    photodiodeValue2 = analogRead(26);
}




void batteryAcquire() {
    // Read analog value (0-1023 on Teensy)
    int rawValue = analogRead(24);
    
    // Convert to voltage (0-3.3V range)
    float adcVoltage = (rawValue / 1023.0f) * 3.3f;
    
    // Scale back up using voltage divider ratio
    // For 3S battery (max ~12.6V), typical divider is R1=10kΩ, R2=3.3kΩ
    // Divider ratio = R2/(R1+R2) = 3.3/(10+3.3) = 0.248
    // So: batteryVoltage = adcVoltage / 0.248
    
    // IMPORTANT: Adjust this ratio based on your actual voltage divider circuit!
    const float VOLTAGE_DIVIDER_RATIO = 2400.0/(2400.0 + 9500.0); // 3.3kΩ / (10kΩ + 3.3kΩ)
    
    batteryVoltage = adcVoltage / VOLTAGE_DIVIDER_RATIO;
    
    // Optional: Apply smoothing filter to reduce noise
    static float filteredVoltage = 0.0f;
    const float FILTER_ALPHA = 0.1f; // Low-pass filter coefficient
    filteredVoltage = (FILTER_ALPHA * batteryVoltage) + ((1.0f - FILTER_ALPHA) * filteredVoltage);
    batteryVoltage = filteredVoltage;
    
    // Clamp to reasonable 3S battery range (9.0V - 12.6V)
    if (batteryVoltage < 9.0f) batteryVoltage = 9.0f;
    if (batteryVoltage > 12.6f) batteryVoltage = 12.6f;
}

float getBatteryVoltage() {
    Serial.println(batteryVoltage);
    return batteryVoltage;
}


int getBatteryPercentage() {
    // Simple linear mapping for 3S LiPo
    // Full: 12.6V (4.2V per cell)
    // Empty: 9.9V (3.3V per cell)
    
    const float MIN_VOLTAGE = 9.9f;
    const float MAX_VOLTAGE = 12.6f;
    
    float percentage = ((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0f;
    
    if (percentage < 0) percentage = 0;
    if (percentage > 100) percentage = 100;
    
    return (int)percentage;
}

int MAVsetup(){
    // Use the same baud rate as your working test
    mavlink.begin(921600);
    
    // Give some time for initialization
    delay(1000);
    
    Serial.println("MAVLink decoder initialized");
    return 1;
}

void MAVLinkAcquire(){
    static unsigned long lastDebug = 0;
    static unsigned long lastRequest = 0;
    
    // Process messages
    while (Serial2.available() > 0) {
        bool messageReceived = mavlink.update();
        
        if (messageReceived) {
            // Update ALL message data
            mavlink.getAttitude(message.roll, message.pitch, message.yaw);
            mavlink.getGPSInfo(message.lat, message.lon, message.alt, message.satellites);
            mavlink.getBatteryInfo(message.voltage, message.current, message.remaining);
            mavlink.getVfrHudData(message.airspeed, message.groundspeed, message.heading, 
                                 message.throttle, message.alt_vfr, message.climb);
            mavlink.getHighResImu(message.acc_x, message.acc_y, message.acc_z, 
                                 message.xgyro, message.ygyro, message.zgyro,
                                 message.xmag, message.ymag, message.zmag, 
                                 message.abs_pressure, message.diff_pressure, 
                                 message.temperature, message.pressure_alt);
            mavlink.getLoggingStats(message.write_rate, message.space_left);
            mavlink.getVibrationData(message.vibe_x, message.vibe_y, message.vibe_z, 
                                    message.clipping_x, message.clipping_y, message.clipping_z);
            
            // SIMPLIFIED: Direct GPS time access without caching or validation
            uint64_t gps_time = 0;
            uint8_t fix_type = 0;
            uint64_t sys_time = 0;
            uint32_t boot_time = 0;
            if (mavlink.getSystemTime(sys_time, boot_time)){
                message.unix_time_usec = sys_time;
            } else {
                message.unix_time_usec = 0;
            }

            // uint64_t gps_time = 0;
            // if (mavlink.getGpsTime(gps_time, fix_type)){
            //     message.unix_time_usec = gps_time;
            // } else {
            //     message.unix_time_usec = 0;
            // }
            
            
            // Always try to get boot time
            if (message.boot_time_ms == 0) {
                uint64_t dummy_time = 0;
                mavlink.getSystemTime(dummy_time, message.boot_time_ms);
            }
        }
    }
    
    // Request data streams periodically
    if (millis() - lastRequest > 5000) {
        mavlink.requestAllDataStreams(10);
        lastRequest = millis();
    }
    
    // Simple debug output
    if (millis() - lastDebug > 5000) {
        Serial.print("MAVLink: Current GPS time ");
        Serial.print((unsigned long)(message.unix_time_usec / 1000000));
        Serial.println(" sec");
        lastDebug = millis();
    }
}

void debugPixhawkData(MavLinkMessage &message){
    // Remove mutex - just read the data directly
    Serial.println("=== Pixhawk Data ===");
    Serial.print("Roll: "); Serial.print(message.roll * 57.3, 2); Serial.println(" deg");
    Serial.print("Pitch: "); Serial.print(message.pitch * 57.3, 2); Serial.println(" deg");
    Serial.print("Yaw: "); Serial.print(message.yaw * 57.3, 2); Serial.println(" deg");
    Serial.print("Latitude: "); Serial.print(message.lat / 1.0e7f, 7); Serial.println(" deg");
    Serial.print("Longitude: "); Serial.print(message.lon / 1.0e7f, 7); Serial.println(" deg");
    Serial.print("Altitude: "); Serial.print(message.alt_vfr, 2); Serial.println(" m");
    Serial.print("Airspeed: "); Serial.print(message.airspeed, 2); Serial.println(" m/s");
    Serial.print("Groundspeed: "); Serial.print(message.groundspeed, 2); Serial.println(" m/s");
    Serial.print("Heading: "); Serial.print(message.heading); Serial.println(" deg");
    Serial.print("Battery Voltage: "); Serial.print(message.voltage, 2); Serial.println(" V");
    Serial.print("Battery Current: "); Serial.print(message.current, 2); Serial.println(" A");
    Serial.print("Remaining Battery: "); Serial.print(message.remaining); Serial.println(" %");
    
    // High-res IMU data
    if (message.acc_x != 0 || message.acc_y != 0 || message.acc_z != 0) {
        Serial.println("--- High-Res IMU ---");
        Serial.print("Accel X: "); Serial.print(message.acc_x, 4); Serial.println(" m/s²");
        Serial.print("Accel Y: "); Serial.print(message.acc_y, 4); Serial.println(" m/s²");
        Serial.print("Accel Z: "); Serial.print(message.acc_z, 4); Serial.println(" m/s²");
        Serial.print("Abs Pressure: "); Serial.print(message.abs_pressure, 2); Serial.println(" hPa");
        Serial.print("Temperature: "); Serial.print(message.temperature, 2); Serial.println(" °C");
    }

    // System time data
    Serial.println("--- System Time ---");
    Serial.print("Unix Time (usec): "); Serial.print(message.unix_time_usec); Serial.println();
    Serial.print("Boot Time (ms): "); Serial.print(message.boot_time_ms); Serial.println();
    
    Serial.println("========================");
}

bool getMavlinkTime(uint64_t &gps_time_usec, uint32_t &boot_time_ms){
    // SIMPLIFIED: Direct GPS time access without validation
    uint8_t fix_type;
    if (mavlink.getGpsTime(gps_time_usec, fix_type)) {
        // GPS time available - get boot time separately
        uint64_t dummy_time;
        mavlink.getSystemTime(dummy_time, boot_time_ms);
        return true;
    }
    
    // Fallback to system time if GPS time not available
    bool success = mavlink.getSystemTime(gps_time_usec, boot_time_ms);
    return success;
}

void mavlinkUpdateThread(){
    while(1){
        // mavlink.update();

        // if (millis() % 5000 < 50) {
        //     mavlink.requestAllDataStreams(5); // Request at 10 Hz
        // }
        MAVLinkAcquire();
    }
}


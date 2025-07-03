#ifndef UTILS_H
#define UTILS_H

#include <TeensyThreads.h>
#include <SD.h>
#include "Waveshare_10Dof-D.h"
#include "MAVLink.h"  // Make sure to include this for MavLinkMessage

#define GPS_SERIAL Serial8
#define TERMINATION_PIN 29

// Define this macro to use altRadioPacket, otherwise radioPacket will be used
// #define USE_ALT_PACKET

// Global IMU data structures
extern IMU_ST_ANGLES_DATA stAngles;
extern IMU_ST_SENSOR_DATA stGyroRawData;
extern IMU_ST_SENSOR_DATA stAccelRawData;
extern IMU_ST_SENSOR_DATA stMagnRawData;
extern Threads::Mutex DAQmutex;




// IMPORTANT: Move MavLinkMessage struct definition here, before it's used
struct MavLinkMessage {
    // Attitude data
    float roll;
    float pitch;
    float yaw;
    // GPS data
    int32_t lat;
    int32_t lon;
    int32_t alt;
    uint8_t satellites;
    // battery info
    float voltage;
    float current;
    int8_t remaining;
    // VHR HUD data
    float airspeed;
    float groundspeed;
    float heading;
    float throttle;
    float alt_vfr;
    float climb;
    // HIGH-RES IMU data
    float acc_x;
    float acc_y;
    float acc_z;
    float xgyro;
    float ygyro;
    float zgyro;
    float xmag;
    float ymag;
    float zmag;
    float abs_pressure;
    float diff_pressure;
    float temperature;
    float pressure_alt;
    // System Time
    uint64_t unix_time_usec;
    uint32_t boot_time_ms;
    // Logging Status
    bool logging_active;
    uint32_t write_rate;
    uint32_t space_left;
    // Arm status
    bool armed; // missing implementation in MavlinkDecoder
    // Vibration data
    float vibe_x;
    float vibe_y;
    float vibe_z;
    uint32_t clipping_x;
    uint32_t clipping_y;
    uint32_t clipping_z;
};

// Declare the global message variable
extern MavLinkMessage message;
extern Threads::Mutex MAVLinkMutex;

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
int SDSetup();
void SDWrite(const String& data);
int sensorSetup();
void calibrateIMU();
void DAQacquire();
void GPSacquire();
void photodiodeAcquire();
void photodiodeSetup();
void batteryAcquire();
int getBatteryPercentage();
void isBatteryLow();
void isBatteryCritical();
float getBatteryVoltage();

void LEDHandler();
void trackingLED();
void sourceLED();
void trackingLEDCached(uint64_t gps_time_usec);
void sourceLEDCached(uint64_t gps_time_usec);
void LED_Setup();

void mavlinkUpdateThread();

extern float batteryVoltage;

// GPS parsing functions
bool parseGPSString(const char* gpsString, GPSData& data);
bool parseGPRMC(const char* rmcString, GPSData& data);
bool parseGPGGA(const char* ggaString, GPSData& data);

// Radio packet functions
void formRadioPacket(char* packet, size_t packet_size);
void updateRadioPacket(int rssi = 0, int snr = 0);

// New functions for altRadioPacket
void formAltRadioPacket(char* packet, size_t packet_size);
void updateAltRadioPacket(int rssi = 0, int snr = 0);

// Pixhawk functions
int MAVsetup();
void MAVLinkAcquire();

//Accessor
bool getMavlinkTime(uint64_t &gps_time_usec, uint32_t &boot_time_ms);

// Debug data functions
void debugPixhawkData(MavLinkMessage &message);

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

struct altRadioPacket {
    // Communication data
    int ack;
    int16_t RSSI;
    int SNR;
    // Time (FC)
    uint64_t fc_unix_time_usec; // IMPORTANT: Example rename from unix_time_usec
    uint32_t fc_boot_time_ms;   // IMPORTANT: Example rename from boot_time_ms
    // GPS data 1 (FC)
    // float gpsLat1;
    // float gpsLon1;
    // float gpsAlt1;
    // float gpsSpeed1;
    // float gpsTime1;
    // GPS data 2 (Pixhawk)
    float gpsLat2;
    float gpsLon2;
    float gpsAlt2;
    float gpsSpeed2;
    uint32_t gpsTime2;
    // IMU data 1 (FC)
    float absPressure1;
    float temperature1;
    float altitude1;
    // IMU data 2 (Pixhawk)
    float absPressure2;
    float temperature2;
    float diffPressure2;
    // FC System status
    bool SDStatus;
    bool actuatorStatus;
    // Pixhawk System status
    bool logging_active;
    uint32_t write_rate;
    uint32_t space_left;
    // Pixhawk System time
    uint64_t pix_unix_time_usec; // IMPORTANT: Example rename from unix_time_usec
    uint32_t pix_boot_time_ms;   // IMPORTANT: Example rename from boot_time_ms
    // Vibration data
    float vibe_x;
    float vibe_y;
    float vibe_z;
    uint32_t clipping_x;
    uint32_t clipping_y;
    uint32_t clipping_z;
    // Navigation data
    float gpsBearing;
    float gpsBearingMagnetic;
    float gpsBearingTrue;
    float gpsBearingGroundSpeed;
    float gpsBearingGroundSpeedMagnetic;
    float gpsBearingGroundSpeedTrue;
    // Photodiode data
    int photodiodeValue1;
    int photodiodeValue2;
    // Battery data
    float FC_battery_voltage; // FC battery voltage
    float LED_battery_voltage; // LED battery voltage
};

#ifdef USE_ALT_PACKET
extern altRadioPacket currentPacket;
#else
extern radioPacket currentPacket;
#endif

// Global GPS data instance
extern GPSData currentGPSData;
extern Threads::Mutex GPSmutex;

#endif
#ifndef UBLOX_GPS_H
#define UBLOX_GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

class UbloxGPS {
private:
    TinyGPSPlus gps;
    bool gpsAvailable = false;
    unsigned long lastDisplayTime = 0;
    const unsigned long DISPLAY_INTERVAL = 1000; // Update interval in milliseconds
    HardwareSerial* gpsSerial;

    // Internal status tracking
    bool locationValid = false;
    bool dateTimeValid = false;
    
    // Cached GPS values for const-correctness
    double cachedLatitude = 0.0;
    double cachedLongitude = 0.0;
    double cachedAltitude = 0.0;
    double cachedSpeed = 0.0;
    double cachedCourse = 0.0;
    double cachedHDOP = 0.0;
    uint16_t cachedYear = 0;
    uint8_t cachedMonth = 0;
    uint8_t cachedDay = 0;
    uint8_t cachedHour = 0;
    uint8_t cachedMinute = 0;
    uint8_t cachedSecond = 0;
    uint8_t cachedCentisecond = 0;
    uint32_t cachedSatellites = 0;
    uint32_t cachedTimeStamp = 0;

public:
    // Constructor
    UbloxGPS(HardwareSerial& serial);

    // Initialize the GPS module
    void begin(long baudRate = 9600);

    // Update GPS data - call this frequently in your loop
    void update();

    // Check if GPS has a valid fix
    bool hasValidFix() const;

    // Get location data
    double getLatitude() const;
    double getLongitude() const;
    double getAltitude() const;    // Meters above sea level
    
    // Get speed and course data
    double getSpeedKmph() const;
    double getCourseDeg() const;   // Course in degrees (0-359.99)

    // Get date and time
    uint16_t getYear() const;
    uint8_t getMonth() const;
    uint8_t getDay() const;
    uint8_t getHour() const;
    uint8_t getMinute() const;
    uint8_t getSecond() const;
    
    // Get satellites info
    uint32_t getSatellitesVisible() const;
    
    // Get HDOP (horizontal dilution of precision)
    double getHDOP() const;
    
    // Convert to UTC timestamp (seconds since 1970-01-01 00:00:00)
    uint32_t getUTCTime() const;
    
    // Check the health of the GPS connection
    bool isConnected() const;
    
    // Return raw TinyGPS object for advanced usage
    TinyGPSPlus& getRawGPS();
    
    // Utilities for formatting
    String getTimeString() const;
    String getDateString() const;
    String getLocationString() const;
    
    // Update cached UTC time
    void updateCachedTime();
};

#endif // UBLOX_GPS_H
#include "UbloxGPS.h"

UbloxGPS::UbloxGPS(HardwareSerial& serial) {
    gpsSerial = &serial;
}

void UbloxGPS::begin(long baudRate) {
    gpsSerial->begin(baudRate);
    Serial.println("GPS initialized on Serial8");
}

void UbloxGPS::update() {
    // Process all available GPS data
    while (gpsSerial->available() > 0) {
        char c = gpsSerial->read();
        if (gps.encode(c)) {
            // Each time a new complete sentence is received, this will be true
            gpsAvailable = true;
        }
    }
    
    // Update status flags
    locationValid = gps.location.isValid();
    dateTimeValid = gps.date.isValid() && gps.time.isValid();
    
    // Update cached values when valid data is available
    if (gps.location.isValid()) {
        cachedLatitude = gps.location.lat();
        cachedLongitude = gps.location.lng();
    }
    
    if (gps.altitude.isValid()) {
        cachedAltitude = gps.altitude.meters();
    }
    
    if (gps.speed.isValid()) {
        cachedSpeed = gps.speed.kmph();
    }
    
    if (gps.course.isValid()) {
        cachedCourse = gps.course.deg();
    }
    
    if (gps.date.isValid()) {
        cachedYear = gps.date.year();
        cachedMonth = gps.date.month();
        cachedDay = gps.date.day();
    }
    
    if (gps.time.isValid()) {
        cachedHour = gps.time.hour();
        cachedMinute = gps.time.minute();
        cachedSecond = gps.time.second();
        cachedCentisecond = gps.time.centisecond();
    }
    
    if (gps.satellites.isValid()) {
        cachedSatellites = gps.satellites.value();
    }
    
    if (gps.hdop.isValid()) {
        cachedHDOP = gps.hdop.hdop();
    }
    
    // Update cached timestamp if date and time are valid
    if (dateTimeValid) {
        updateCachedTime();
    }
    
    // Periodically print GPS information to Serial for debugging
    unsigned long currentMillis = millis();
    if (gpsAvailable && (currentMillis - lastDisplayTime > DISPLAY_INTERVAL)) {
        lastDisplayTime = currentMillis;
        
        Serial.print("GPS Status: ");
        if (hasValidFix()) {
            Serial.println("Fix acquired!");
            Serial.print("Location: "); Serial.print(getLocationString());
            Serial.print(" Altitude: "); Serial.print(getAltitude(), 1); Serial.println(" meters");
            Serial.print("Time: "); Serial.println(getTimeString());
            Serial.print("Date: "); Serial.println(getDateString());
            Serial.print("Satellites: "); Serial.println(getSatellitesVisible());
        } else {
            Serial.println("Waiting for fix...");
        }
    }
}

bool UbloxGPS::hasValidFix() const {
    return locationValid;
}

double UbloxGPS::getLatitude() const {
    return locationValid ? cachedLatitude : 0.0;
}

double UbloxGPS::getLongitude() const {
    return locationValid ? cachedLongitude : 0.0;
}

double UbloxGPS::getAltitude() const {
    return gps.altitude.isValid() ? cachedAltitude : 0.0;
}

double UbloxGPS::getSpeedKmph() const {
    return gps.speed.isValid() ? cachedSpeed : 0.0;
}

double UbloxGPS::getCourseDeg() const {
    return gps.course.isValid() ? cachedCourse : 0.0;
}

uint16_t UbloxGPS::getYear() const {
    return gps.date.isValid() ? cachedYear : 0;
}

uint8_t UbloxGPS::getMonth() const {
    return gps.date.isValid() ? cachedMonth : 0;
}

uint8_t UbloxGPS::getDay() const {
    return gps.date.isValid() ? cachedDay : 0;
}

uint8_t UbloxGPS::getHour() const {
    return gps.time.isValid() ? cachedHour : 0;
}

uint8_t UbloxGPS::getMinute() const {
    return gps.time.isValid() ? cachedMinute : 0;
}

uint8_t UbloxGPS::getSecond() const {
    return gps.time.isValid() ? cachedSecond : 0;
}

uint32_t UbloxGPS::getSatellitesVisible() const {
    return gps.satellites.isValid() ? cachedSatellites : 0;
}

double UbloxGPS::getHDOP() const {
    return gps.hdop.isValid() ? cachedHDOP : 0.0;
}

bool UbloxGPS::isConnected() const {
    return gpsAvailable;
}

TinyGPSPlus& UbloxGPS::getRawGPS() {
    return gps;
}

String UbloxGPS::getTimeString() const {
    if (!gps.time.isValid()) {
        return "Invalid Time";
    }
    
    char buffer[32];
    sprintf(buffer, "%02d:%02d:%02d.%02d", 
        cachedHour, 
        cachedMinute, 
        cachedSecond,
        cachedCentisecond);
    
    return String(buffer);
}

String UbloxGPS::getDateString() const {
    if (!gps.date.isValid()) {
        return "Invalid Date";
    }
    
    char buffer[32];
    sprintf(buffer, "%04d-%02d-%02d", 
        cachedYear, 
        cachedMonth, 
        cachedDay);
    
    return String(buffer);
}

String UbloxGPS::getLocationString() const {
    if (!locationValid) {
        return "Invalid Location";
    }
    
    char buffer[64];
    sprintf(buffer, "%.6f, %.6f", 
        cachedLatitude,
        cachedLongitude);
    
    return String(buffer);
}

void UbloxGPS::updateCachedTime() {
    // Simple UTC timestamp calculation
    // This is a simplified version and doesn't account for leap years/seconds perfectly
    uint16_t year = cachedYear;
    uint8_t month = cachedMonth;
    uint8_t day = cachedDay;
    uint8_t hour = cachedHour;
    uint8_t minute = cachedMinute;
    uint8_t second = cachedSecond;
    
    // Calculate days since Jan 1, 1970
    uint32_t seconds = (year - 1970) * 365 * 24 * 60 * 60;
    
    // Add leap years
    uint32_t leapYears = (year - 1969) / 4 - (year - 1901) / 100 + (year - 1601) / 400;
    seconds += leapYears * 24 * 60 * 60;
    
    // Add days for months this year
    const uint8_t daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    for (int i = 1; i < month; i++) {
        seconds += daysInMonth[i] * 24 * 60 * 60;
    }
    
    // Add leap day for this year if appropriate
    if (month > 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
        seconds += 24 * 60 * 60;
    }
    
    // Add days this month
    seconds += (day - 1) * 24 * 60 * 60;
    
    // Add hours, minutes, seconds
    seconds += hour * 60 * 60;
    seconds += minute * 60;
    seconds += second;
    
    cachedTimeStamp = seconds;
}

uint32_t UbloxGPS::getUTCTime() const {
    if (!dateTimeValid) {
        return 0;
    }
    
    return cachedTimeStamp;
}
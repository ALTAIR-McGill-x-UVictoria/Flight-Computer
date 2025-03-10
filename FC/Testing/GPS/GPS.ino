#include <SoftwareSerial.h>

// The serial connection to the GPS module
#define GPSserial Serial8

// GPS data structure
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
bool parseGPSString(const char* gpsString, GPSData& data);
bool parseGPRMC(const char* rmcString, GPSData& data);
bool parseGPGGA(const char* ggaString, GPSData& data);

void setup() {
    Serial.begin(115200);  // Debug output
    GPSserial.begin(9600); // GPS module
}

void loop() {
    static char gpsString[100];
    static int index = 0;
    
    while (GPSserial.available() > 0) {
        char c = GPSserial.read();
        
        // Store character
        if (c != '\n' && index < 99) {
            gpsString[index++] = c;
        }
        // End of line detected
        else if (c == '\n') {
            gpsString[index] = '\0';  // Null terminate
            
            // Parse and print the GPS data
            GPSData data;
            if (parseGPSString(gpsString, data)) {
                if (data.valid) {
                    Serial.println("Valid GPS Fix:");
                    Serial.print("Lat: "); Serial.print(data.latitude, 6);
                    Serial.print(" Lon: "); Serial.print(data.longitude, 6);
                    Serial.print(" Alt: "); Serial.print(data.altitude);
                    Serial.print(" Speed: "); Serial.print(data.speed);
                    Serial.print(" Sats: "); Serial.println(data.satellites);
                }
            }
            
            // Also print raw NMEA string for debugging
            Serial.println(gpsString);
            
            // Reset for next line
            index = 0;
        }
    }
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
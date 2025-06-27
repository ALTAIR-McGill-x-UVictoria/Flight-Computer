#include <Arduino.h>
#include "MavlinkDecoder.h"

// Create an instance of the MAVLink decoder
MavlinkDecoder mavlink;

// Timer for periodic operations
unsigned long lastHeartbeat = 0;
unsigned long lastPrint = 0;
bool dataStreamsRequested = false;

// Add these variables near the top with other global variables
bool loggingActive = false;
unsigned long lastLoggingToggle = 0;
unsigned long loggingStartTime = 0;

void setup() {
    // Initialize serial communication with the computer
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial port to connect
    }
    
    Serial.println("MAVLink Decoder Example");
    Serial.println("Send 'L' to toggle logging on/off");
    Serial.println("Send 'A' to arm vehicle and start logging");
    Serial.println("Send 'D' to disarm vehicle and stop logging");
    
    // Initialize the MAVLink decoder with Serial2 at appropriate baud rate
    mavlink.begin(921600);
    
    delay(1000); // Give some time for initialization
}

void loop() {
    // Update the MAVLink decoder (process incoming messages)
    mavlink.update();
    
    // Send heartbeat message every 1 second
    unsigned long currentMillis = millis();
    if (currentMillis - lastHeartbeat > 1000) {
        lastHeartbeat = currentMillis;
        // mavlink.sendHeartbeat();
        
        // Request data streams after a few heartbeats
        if (!dataStreamsRequested && currentMillis > 5000) {
            mavlink.requestAllDataStreams(10); // Request at 10 Hz
            dataStreamsRequested = true;
        }
    }
    
    // Update the serial command handler in loop()

    // Check for serial commands to control logging and arming
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'L' || cmd == 'l') {
            // Toggle logging using the existing logging commands
            if (!loggingActive) {
                if (mavlink.startLogging()) {
                    loggingActive = true;
                    loggingStartTime = currentMillis;
                    Serial.println("Starting onboard logging...");
                }
            } else {
                if (mavlink.stopLogging()) {
                    loggingActive = false;
                    Serial.println("Stopping onboard logging");
                }
            }
            lastLoggingToggle = currentMillis;
        }
        else if (cmd == 'A' || cmd == 'a') {
            // Arm the vehicle to trigger logging
            mavlink.armVehicle(true);
            Serial.println("Sending arm command to trigger logging");
        }
        else if (cmd == 'D' || cmd == 'd') {
            // Disarm the vehicle to stop logging
            mavlink.armVehicle(false);
            Serial.println("Sending disarm command to stop logging");
        }
    }
    
    // Print received data every 2 seconds
    if (currentMillis - lastPrint > 2000) {
        lastPrint = currentMillis;
        
        // Get attitude data
        float roll, pitch, yaw;
        if (mavlink.getAttitude(roll, pitch, yaw)) {
            Serial.println("--- Attitude Data ---");
            Serial.print("Roll: "); Serial.print(roll * 57.3, 2); Serial.println(" deg");
            Serial.print("Pitch: "); Serial.print(pitch * 57.3, 2); Serial.println(" deg");
            Serial.print("Yaw: "); Serial.print(yaw * 57.3, 2); Serial.println(" deg");
        }
        
        // Get GPS data
        int32_t lat, lon, alt;
        uint8_t satellites;
        if (mavlink.getGPSInfo(lat, lon, alt, satellites)) {
            Serial.println("--- GPS Data ---");
            Serial.print("Lat: "); Serial.print(lat / 10000000.0, 7); Serial.println(" deg");
            Serial.print("Lon: "); Serial.print(lon / 10000000.0, 7); Serial.println(" deg");
            Serial.print("Alt: "); Serial.print(alt / 1000.0, 2); Serial.println(" m");
            Serial.print("Satellites: "); Serial.println(satellites);
        }
        
        // Get battery info
        float voltage, current;
        int8_t remaining;
        if (mavlink.getBatteryInfo(voltage, current, remaining)) {
            Serial.println("--- Battery Info ---");
            Serial.print("Voltage: "); Serial.print(voltage, 2); Serial.println(" V");
            Serial.print("Current: "); Serial.print(current, 2); Serial.println(" A");
            Serial.print("Remaining: "); Serial.print(remaining); Serial.println(" %");
        }
        
        // Get VFR HUD data
        float airspeed, groundspeed, heading, throttle, alt_vfr, climb;
        if (mavlink.getVfrHudData(airspeed, groundspeed, heading, throttle, alt_vfr, climb)) {
            Serial.println("--- VFR HUD Data ---");
            Serial.print("Airspeed: "); Serial.print(airspeed, 2); Serial.println(" m/s");
            Serial.print("Groundspeed: "); Serial.print(groundspeed, 2); Serial.println(" m/s");
            Serial.print("Heading: "); Serial.print(heading); Serial.println(" deg");
            Serial.print("Throttle: "); Serial.print(throttle); Serial.println(" %");
            Serial.print("Altitude: "); Serial.print(alt_vfr, 2); Serial.println(" m");
            Serial.print("Climb Rate: "); Serial.print(climb, 2); Serial.println(" m/s");
        }
        
        // Get RC channel data
        uint16_t channels[18];
        uint8_t chancount;
        if (mavlink.getRcChannels(channels, chancount)) {
            Serial.println("--- RC Channels ---");
            for (int i = 0; i < chancount && i < 8; i++) {  // Show first 8 channels
                Serial.print("CH");
                Serial.print(i+1);
                Serial.print(": ");
                Serial.println(channels[i]);
            }
        }
        
        // Get high-resolution IMU data
        float xacc, yacc, zacc;
        float xgyro, ygyro, zgyro;
        float xmag, ymag, zmag;
        float abs_pressure, diff_pressure, temperature;
        
        if (mavlink.getHighResImu(xacc, yacc, zacc, xgyro, ygyro, zgyro, 
                                 xmag, ymag, zmag, abs_pressure, diff_pressure, temperature)) {
            Serial.println("--- High-Resolution IMU Data ---");
            
            // Accelerometer data (m/s²)
            Serial.println("Accelerometer (m/s²):");
            Serial.print("  X: "); Serial.print(xacc, 4);
            Serial.print("  Y: "); Serial.print(yacc, 4);
            Serial.print("  Z: "); Serial.println(zacc, 4);
            
            // Gyroscope data (rad/s, convert to deg/s for display)
            Serial.println("Gyroscope (deg/s):");
            Serial.print("  X: "); Serial.print(xgyro * 57.3, 4);
            Serial.print("  Y: "); Serial.print(ygyro * 57.3, 4);
            Serial.print("  Z: "); Serial.println(zgyro * 57.3, 4);
            
            // Magnetometer data (gauss)
            Serial.println("Magnetometer (gauss):");
            Serial.print("  X: "); Serial.print(xmag, 4);
            Serial.print("  Y: "); Serial.print(ymag, 4);
            Serial.print("  Z: "); Serial.println(zmag, 4);
            
            // Pressure and temperature
            Serial.print("Absolute Pressure: "); Serial.print(abs_pressure, 2); Serial.println(" hPa");
            Serial.print("Differential Pressure: "); Serial.print(diff_pressure, 4); Serial.println(" hPa");
            Serial.print("Temperature: "); Serial.print(temperature, 2); Serial.println(" °C");
        }
        
        // Get system time data
        uint64_t unix_time_usec;
        uint32_t boot_time_ms;
        if (mavlink.getSystemTime(unix_time_usec, boot_time_ms)) {
            Serial.println("--- System Time ---");
            
            // Format and display Unix time (convert to seconds)
            time_t unix_time_sec = unix_time_usec / 1000000;
            Serial.print("UTC Time: ");
            
            // Calculate hours, minutes, seconds
            int hours = (unix_time_sec % 86400) / 3600;
            int minutes = (unix_time_sec % 3600) / 60;
            int seconds = unix_time_sec % 60;
            
            // Format as HH:MM:SS
            if (hours < 10) Serial.print("0");
            Serial.print(hours);
            Serial.print(":");
            if (minutes < 10) Serial.print("0");
            Serial.print(minutes);
            Serial.print(":");
            if (seconds < 10) Serial.print("0");
            Serial.println(seconds);
            
            // Display time since boot
            unsigned long boot_seconds = boot_time_ms / 1000;
            unsigned long boot_minutes = boot_seconds / 60;
            unsigned long boot_hours = boot_minutes / 60;
            
            Serial.print("System Uptime: ");
            Serial.print(boot_hours);
            Serial.print("h ");
            Serial.print(boot_minutes % 60);
            Serial.print("m ");
            Serial.print(boot_seconds % 60);
            Serial.println("s");
        }
        
        // Get vibration data
        float vibe_x, vibe_y, vibe_z;
        uint32_t clip_x, clip_y, clip_z;
        if (mavlink.getVibrationData(vibe_x, vibe_y, vibe_z, clip_x, clip_y, clip_z)) {
            Serial.println("--- Vibration Data ---");
            Serial.print("X: "); Serial.print(vibe_x, 3); 
            Serial.print(" Y: "); Serial.print(vibe_y, 3); 
            Serial.print(" Z: "); Serial.println(vibe_z, 3);
            
            // Only show clipping if it's happening (non-zero values)
            if (clip_x > 0 || clip_y > 0 || clip_z > 0) {
                Serial.println("Accel Clipping:");
                Serial.print("X: "); Serial.print(clip_x);
                Serial.print(" Y: "); Serial.print(clip_y);
                Serial.print(" Z: "); Serial.println(clip_z);
            }
        }
        
        // Display logging status
        if (loggingActive) {
            Serial.println("--- Logging Status ---");
            Serial.println("Logging: ACTIVE");
            
            // Calculate logging duration
            unsigned long logDuration = currentMillis - loggingStartTime;
            unsigned long logSeconds = logDuration / 1000;
            unsigned long logMinutes = logSeconds / 60;
            unsigned long logHours = logMinutes / 60;
            
            Serial.print("Logging Duration: ");
            Serial.print(logHours);
            Serial.print("h ");
            Serial.print(logMinutes % 60);
            Serial.print("m ");
            Serial.print(logSeconds % 60);
            Serial.println("s");
            
            // Get and display logging statistics if available
            uint32_t write_rate, space_left;
            if (mavlink.getLoggingStats(write_rate, space_left)) {
                Serial.print("Write Rate: "); 
                Serial.print(write_rate / 1024.0, 2); 
                Serial.println(" KB/s");
                
                Serial.print("Space Left: ");
                if (space_left > 1024) {
                    Serial.print(space_left / 1024.0, 2);
                    Serial.println(" MB");
                } else {
                    Serial.print(space_left);
                    Serial.println(" KB");
                }
            }
        } else if (currentMillis - lastLoggingToggle < 5000) {
            // Show status briefly after toggling
            Serial.println("--- Logging Status ---");
            Serial.println("Logging: INACTIVE");
        }
        
        Serial.println();
        Serial.println("Send 'L' to toggle logging on/off");
    }
}
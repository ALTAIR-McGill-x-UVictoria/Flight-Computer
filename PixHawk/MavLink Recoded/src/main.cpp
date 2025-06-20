#include <Arduino.h>
#include <Teensy_PWM.h>
#include "MavlinkDecoder.h"

// Define ESC parameters
constexpr uint8_t ESC_PIN       = 24;      // PWM output pin
constexpr float   ESC_FREQUENCY = 50.0f;   // Standard hobby-ESC rate (50 Hz)
constexpr float   MIN_DUTY      = 5.0f;    // 1 ms pulse  ≈ throttle off
constexpr float   MAX_DUTY      = 10.0f;   // 2 ms pulse  ≈ full throttle
constexpr uint32_t FULL_RUN_MS  = 200000;

// Create an instance of the MAVLink decoder
MavlinkDecoder mavlink;
// Create an instance of the esc
Teensy_PWM* escPWM;

bool motorRunning = false;
unsigned long motorStartTime = 0;
bool motorHasRun = false;

unsigned long loggingStartTime = 0; // Start time for logging


// Timer for periodic operations
unsigned long lastHeartbeat = 0;
unsigned long lastPrint = 0;
bool dataStreamsRequested = false;

// --- Logging Buffer Definitions ---
constexpr size_t LOG_BUFFER_SIZE = 128; // Number of lines to buffer (adjust as needed)
constexpr size_t LOG_LINE_MAXLEN = 48;  // Max chars per log line

char logBuffer[LOG_BUFFER_SIZE][LOG_LINE_MAXLEN];
size_t logHead = 0;
size_t logTail = 0;

// Add a helper to buffer log lines
void bufferLog(const char* line) {
    strncpy(logBuffer[logHead], line, LOG_LINE_MAXLEN - 1);
    logBuffer[logHead][LOG_LINE_MAXLEN - 1] = '\0';
    logHead = (logHead + 1) % LOG_BUFFER_SIZE;
    // If buffer is full, overwrite oldest
    if (logHead == logTail) logTail = (logTail + 1) % LOG_BUFFER_SIZE;
}

// Flush buffer to Serial
void flushLogBuffer() {
    while (logTail != logHead) {
        Serial.println(logBuffer[logTail]);
        logTail = (logTail + 1) % LOG_BUFFER_SIZE;
    }
}

void setup() {
    loggingStartTime = micros();
    Serial.begin(15200);
    while (!Serial);

    Serial.println("Initializing...");
    mavlink.begin(921600);
    // mavlink.begin(115200);
    delay(1000);

    escPWM = new Teensy_PWM(ESC_PIN, ESC_FREQUENCY, MIN_DUTY);
    escPWM->setPWM(ESC_PIN, ESC_FREQUENCY, MIN_DUTY);

    // Serial.println("Setup complete. Will spin motor from loop()");

}


void loop() {
    // Update the MAVLink decoder (process incoming messages)
    mavlink.requestAllDataStreams(10); // or specifically request SYSTEM_TIME
    mavlink.update();

    // Send heartbeat message every 1 second
    unsigned long currentMillis = millis();
    unsigned long currentMicros = micros();

    // --- Add 10s delay after logging starts before running motor ---
    

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

    if (currentMicros - lastPrint > 1000) {
        lastPrint = currentMicros;

        float roll, pitch, yaw;
        if (mavlink.getAttitude(roll, pitch, yaw)) {
            // char line[LOG_LINE_MAXLEN];

            // snprintf(line, LOG_LINE_MAXLEN, "%lu,Roll,%.2f", micros(), roll * 57.3f);
            // bufferLog(line);
            // snprintf(line, LOG_LINE_MAXLEN, "%lu,Pitch,%.2f", micros(), pitch * 57.3f);
            // bufferLog(line);
            // snprintf(line, LOG_LINE_MAXLEN, "%lu,Yaw,%.2f", micros(), yaw * 57.3f);
            // bufferLog(line);
            Serial.print(micros()); Serial.print(","); Serial.print("roll;"); Serial.println(roll * 57.3f);
            Serial.print(micros()); Serial.print(","); Serial.print("pitch;"); Serial.println(pitch * 57.3f);
            Serial.print(micros()); Serial.print(","); Serial.print("yaw;"); Serial.println(yaw * 57.3f);
        
        }

        if (!motorRunning && !motorHasRun && (micros() - loggingStartTime >= 10000000)) {
        motorRunning = true;
        motorStartTime = currentMicros;


        // Buffer motor start log
        // char motorStartLine[LOG_LINE_MAXLEN];
        // snprintf(motorStartLine, LOG_LINE_MAXLEN, "%lu,Motor,START", micros());
        // bufferLog(motorStartLine);
        Serial.flush();
        // Print unix time (seconds) before Motor,START
        Serial.print(motorStartTime); Serial.print(","); Serial.println("Motor");
        escPWM->setPWM(ESC_PIN, ESC_FREQUENCY, 0.7 * MAX_DUTY);
        motorHasRun = true;
        tone(20, 440, 200);
    }

    if (motorRunning && (currentMicros - motorStartTime >= FULL_RUN_MS)) {
        // Buffer motor stop log
        // char motorStopLine[LOG_LINE_MAXLEN];
        // snprintf(motorStopLine, LOG_LINE_MAXLEN, "%lu,Motor,STOP", micros());
        // bufferLog(motorStopLine);
        // Serial.flush();
        Serial.print(micros()); Serial.print(","); Serial.println("Motor");
        // Serial.print("Time (ms): "); Serial.println(currentMillis);
        escPWM->setPWM(ESC_PIN, ESC_FREQUENCY, MIN_DUTY);
        motorRunning = false;
        noTone(20);
    }
    }

    // Flush buffer at the end of each loop
    flushLogBuffer();

    // // Get system time data
    // uint64_t unix_time_usec;
    // uint32_t boot_time_ms;
    // if (mavlink.getSystemTime(unix_time_usec, boot_time_ms)) {
    //     Serial.println("--- System Time ---");
        
    //     // Format and display Unix time (convert to seconds)
    //     time_t unix_time_sec = unix_time_usec / 1000000;
    //     Serial.print("UTC Time: ");
        
    //     // Calculate hours, minutes, seconds
    //     int hours = (unix_time_sec % 86400) / 3600;
    //     int minutes = (unix_time_sec % 3600) / 60;
    //     int seconds = unix_time_sec % 60;
        
    //     // Format as HH:MM:SS
    //     if (hours < 10) Serial.print("0");
    //     Serial.print(hours);
    //     Serial.print(":");
    //     if (minutes < 10) Serial.print("0");
    //     Serial.print(minutes);
    //     Serial.print(":");
    //     if (seconds < 10) Serial.print("0");
    //     Serial.println(seconds);
        
    //     // Display time since boot
    //     unsigned long boot_seconds = boot_time_ms / 1000;
    //     unsigned long boot_minutes = boot_seconds / 60;
    //     unsigned long boot_hours = boot_minutes / 60;
        
    //     Serial.print("System Uptime: ");
    //     Serial.print(boot_hours);
    //     Serial.print("h ");
    //     Serial.print(boot_minutes % 60);
    //     Serial.print("m ");
    //     Serial.print(boot_seconds % 60);
    //     Serial.println("s");
    // }
    
    
    // // Display logging status
    // if (loggingActive) {
    //     Serial.println("--- Logging Status ---");
    //     Serial.println("Logging: ACTIVE");
        
    //     // Calculate logging duration
    //     unsigned long logDuration = currentMillis - loggingStartTime;
    //     unsigned long logSeconds = logDuration / 1000;
    //     unsigned long logMinutes = logSeconds / 60;
    //     unsigned long logHours = logMinutes / 60;
        
    //     Serial.print("Logging Duration: ");
    //     Serial.print(logHours);
    //     Serial.print("h ");
    //     Serial.print(logMinutes % 60);
    //     Serial.print("m ");
    //     Serial.print(logSeconds % 60);
    //     Serial.println("s");
        
    //     // Get and display logging statistics if available
    //     uint32_t write_rate, space_left;
    //     if (mavlink.getLoggingStats(write_rate, space_left)) {
    //         Serial.print("Write Rate: "); 
    //         Serial.print(write_rate / 1024.0, 2); 
    //         Serial.println(" KB/s");
            
    //         Serial.print("Space Left: ");
    //         if (space_left > 1024) {
    //             Serial.print(space_left / 1024.0, 2);
    //             Serial.println(" MB");
    //         } else {
    //             Serial.print(space_left);
    //             Serial.println(" KB");
    //         }
    //     }
    // } else if (currentMillis - lastLoggingToggle < 5000) {
    //     // Show status briefly after toggling
    //     Serial.println("--- Logging Status ---");
    //     Serial.println("Logging: INACTIVE");
    // }
    
    // Serial.println();
    // Serial.println("Send 'L' to toggle logging on/off");
}
#ifndef MAVLINK_DECODER_H
#define MAVLINK_DECODER_H

#include <Arduino.h>
// Include the MAVLink library
#include <mavlink/common/mavlink.h>

class MavlinkDecoder {
private:
    // MAVLink message and status variables
    mavlink_message_t msg;
    mavlink_status_t status;
    
    // System and component IDs
    uint8_t systemId = 1;
    uint8_t componentId = 1;
    uint8_t targetSystem = 1;
    uint8_t targetComponent = 0;
    
    // Buffer for incoming data
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
    // Helper method to specifically request HIGHRES_IMU messages
    void requestHighResIMU();

    // Helper method to request logging parameters
    void requestLogParameters();

    // Helper method to send shell commands to Pixhawk
    void sendShellCommand(const char* command);

public:
    // Constructor
    MavlinkDecoder();
    
    // Initialize Serial2 for MAVLink communication
    void begin(long baudRate = 57600);
    
    // Process incoming MAVLink messages
    bool update();
    
    // Send a heartbeat message
    void sendHeartbeat();
    
    // Request all data streams from the flight controller
    void requestAllDataStreams(uint8_t streamRate = 10);
    
    // Get last received attitude data
    bool getAttitude(float &roll, float &pitch, float &yaw);
    
    // Get GPS data
    bool getGPSInfo(int32_t &lat, int32_t &lon, int32_t &alt, uint8_t &satellites);
    
    // Get battery information
    bool getBatteryInfo(float &voltage, float &current, int8_t &remaining);
    
    // Get VFR HUD data (airspeed, groundspeed, heading, etc.)
    bool getVfrHudData(float &airspeed, float &groundspeed, float &heading, 
                      float &throttle, float &alt, float &climb);
                      
    // Get RC input channels
    bool getRcChannels(uint16_t* channels, uint8_t &chancount);
    
    // Get high-resolution IMU data
    bool getHighResImu(float &xacc, float &yacc, float &zacc, 
                      float &xgyro, float &ygyro, float &zgyro,
                      float &xmag, float &ymag, float &zmag,
                      float &abs_pressure, float &diff_pressure, float &temperature, float &pressure_alt);

    // Add this to the public section of your MavlinkDecoder class
    void requestSpecificStreams();
    
    // Get system time data
    bool getSystemTime(uint64_t &unix_time_usec, uint32_t &boot_time_ms);

    // Start logging data to the Pixhawk's SD card
    bool startLogging();

    // Stop logging data to the Pixhawk's SD card
    bool stopLogging();

    // Check if logging is active
    bool getLoggingStatus();

    // Get logging statistics
    bool getLoggingStats(uint32_t &write_rate, uint32_t &space_left);

    // Arm or disarm the vehicle (which will trigger logging when armed)
    bool armVehicle(bool arm = true);

    // Get vibration data
    bool getVibrationData(float &vibe_x, float &vibe_y, float &vibe_z, 
                          uint32_t &clip_x, uint32_t &clip_y, uint32_t &clip_z);
};

#endif // MAVLINK_DECODER_H
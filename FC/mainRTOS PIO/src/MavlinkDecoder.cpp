#include "MavlinkDecoder.h"

// Store the last received data
static struct {
    float roll, pitch, yaw;
    bool attitude_valid = false;
    
    // GPS data
    int32_t lat, lon, alt;
    uint8_t satellites;
    uint64_t gps_time_usec;  // GPS timestamp in microseconds
    uint8_t gps_fix_type;    // GPS fix type (0-1: no fix, 2: 2D fix, 3: 3D fix)
    bool gps_valid = false;
    bool gps_time_valid = false;  // Flag for valid GPS time
    
    float voltage, current;
    int8_t remaining;
    bool battery_valid = false;
    
    // VFR HUD data
    float airspeed, groundspeed, heading, throttle, alt_vfr, climb;
    bool vfr_valid = false;
    
    // RC Channels data (up to 18 channels)
    uint16_t rc_channels[18];
    uint8_t chancount;
    bool rc_valid = false;
    
    // System status
    uint32_t onboard_control_sensors_present;
    uint32_t onboard_control_sensors_enabled;
    uint32_t onboard_control_sensors_health;
    bool system_status_valid = false;
    
    // Global position
    int32_t relative_alt;
    int16_t vx, vy, vz;
    uint16_t hdg;
    bool global_position_valid = false;
    
    // Raw IMU data
    int16_t xacc, yacc, zacc;
    int16_t xgyro, ygyro, zgyro;
    int16_t xmag, ymag, zmag;
    bool raw_imu_valid = false;
    
    // Barometer data
    float press_abs, press_diff;
    int16_t temperature;
    bool scaled_pressure_valid = false;
    
    // Home position
    int32_t home_lat, home_lon, home_alt;
    bool home_valid = false;
    
    // High-resolution IMU data
    float highres_xacc, highres_yacc, highres_zacc;
    float highres_xgyro, highres_ygyro, highres_zgyro;
    float highres_xmag, highres_ymag, highres_zmag;
    float highres_abs_pressure, highres_diff_pressure, highres_temperature, pressure_alt;
    bool highres_imu_valid = false;
    
    // System time data
    uint64_t time_unix_usec;  // Unix timestamp in microseconds
    uint32_t time_boot_ms;    // Milliseconds since boot
    bool system_time_valid = false;
    
    // Logging status
    bool logging_active = false;
    uint32_t logging_write_rate = 0;
    uint32_t logging_space_left = 0;
    bool logging_status_valid = false;
    
    // Vibration data
    float vibration_x;        // Vibration levels on X axis
    float vibration_y;        // Vibration levels on Y axis
    float vibration_z;        // Vibration levels on Z axis
    uint32_t clipping_x;      // Accelerometer clipping count for X axis
    uint32_t clipping_y;      // Accelerometer clipping count for Y axis
    uint32_t clipping_z;      // Accelerometer clipping count for Z axis
    bool vibration_valid = false;
    
} mavlink_data;

MavlinkDecoder::MavlinkDecoder() {
}

void MavlinkDecoder::begin(long baudRate) {
    Serial2.begin(baudRate);
    // // Serial.println("MAVLink decoder initialized");
}

bool MavlinkDecoder::update() {
    bool messageReceived = false;
    static unsigned long lastHighresImuDebug = 0;
    
    // Check for available data on Serial2
    while (Serial2.available() > 0) {
        uint8_t c = Serial2.read();
        // Serial.println("here");
        
        // Try to parse the MAVLink message
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            messageReceived = true;
            
            // Uncomment for debug - will show all received message IDs
            // // Serial.print("Received message ID: ");
            // // Serial.println(msg.msgid);
            
            // Debug for HIGHRES_IMU specifically
            if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
                if (millis() - lastHighresImuDebug > 5000) {  // Show debug every 5 seconds
                    lastHighresImuDebug = millis();
                    // Serial.println("HIGHRES_IMU message received!");
                }
            }
            
            // Handle the received message based on its ID
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    // Process heartbeat
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                    
                    // Store target system/component for future communication
                    if (msg.sysid != systemId) {
                        targetSystem = msg.sysid;
                        targetComponent = msg.compid;
                    }
                    break;
                }
                
                case MAVLINK_MSG_ID_ATTITUDE: {
                    // Process attitude data
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    
                    // Store attitude data
                    mavlink_data.roll = attitude.roll;
                    mavlink_data.pitch = attitude.pitch;
                    mavlink_data.yaw = attitude.yaw;
                    mavlink_data.attitude_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_GPS_RAW_INT: {
                    // Process GPS data
                    mavlink_gps_raw_int_t gps;
                    mavlink_msg_gps_raw_int_decode(&msg, &gps);
                    
                    // Store GPS data
                    mavlink_data.lat = gps.lat;
                    mavlink_data.lon = gps.lon;
                    mavlink_data.alt = gps.alt;
                    mavlink_data.satellites = gps.satellites_visible;
                    mavlink_data.gps_fix_type = gps.fix_type;
                    mavlink_data.gps_time_usec = gps.time_usec;  // FIXED: Store GPS timestamp
                    mavlink_data.gps_valid = true;
                    mavlink_data.gps_time_valid = (gps.time_usec > 0); // FIXED: Mark time as valid if non-zero
                    break;
                }
                
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    // Process system status for battery info
                    mavlink_sys_status_t sys_status;
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
                    
                    // Store battery data
                    mavlink_data.voltage = sys_status.voltage_battery / 1000.0f; // Convert to volts
                    mavlink_data.current = sys_status.current_battery / 100.0f;  // Convert to amps
                    mavlink_data.remaining = sys_status.battery_remaining;
                    mavlink_data.battery_valid = true;
                    
                    // Store system status data
                    mavlink_data.onboard_control_sensors_present = sys_status.onboard_control_sensors_present;
                    mavlink_data.onboard_control_sensors_enabled = sys_status.onboard_control_sensors_enabled;
                    mavlink_data.onboard_control_sensors_health = sys_status.onboard_control_sensors_health;
                    mavlink_data.system_status_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_VFR_HUD: {
                    // Process VFR HUD data
                    mavlink_vfr_hud_t vfr;
                    mavlink_msg_vfr_hud_decode(&msg, &vfr);
                    
                    // Store VFR HUD data
                    mavlink_data.airspeed = vfr.airspeed;
                    mavlink_data.groundspeed = vfr.groundspeed;
                    mavlink_data.heading = vfr.heading;
                    mavlink_data.throttle = vfr.throttle;
                    mavlink_data.alt_vfr = vfr.alt;
                    mavlink_data.climb = vfr.climb;
                    mavlink_data.vfr_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_RC_CHANNELS: {
                    // Process RC channels data
                    mavlink_rc_channels_t rc;
                    mavlink_msg_rc_channels_decode(&msg, &rc);
                    
                    // Store RC channel data
                    mavlink_data.chancount = rc.chancount;
                    mavlink_data.rc_channels[0] = rc.chan1_raw;
                    mavlink_data.rc_channels[1] = rc.chan2_raw;
                    mavlink_data.rc_channels[2] = rc.chan3_raw;
                    mavlink_data.rc_channels[3] = rc.chan4_raw;
                    mavlink_data.rc_channels[4] = rc.chan5_raw;
                    mavlink_data.rc_channels[5] = rc.chan6_raw;
                    mavlink_data.rc_channels[6] = rc.chan7_raw;
                    mavlink_data.rc_channels[7] = rc.chan8_raw;
                    mavlink_data.rc_channels[8] = rc.chan9_raw;
                    mavlink_data.rc_channels[9] = rc.chan10_raw;
                    mavlink_data.rc_channels[10] = rc.chan11_raw;
                    mavlink_data.rc_channels[11] = rc.chan12_raw;
                    mavlink_data.rc_channels[12] = rc.chan13_raw;
                    mavlink_data.rc_channels[13] = rc.chan14_raw;
                    mavlink_data.rc_channels[14] = rc.chan15_raw;
                    mavlink_data.rc_channels[15] = rc.chan16_raw;
                    mavlink_data.rc_channels[16] = rc.chan17_raw;
                    mavlink_data.rc_channels[17] = rc.chan18_raw;
                    mavlink_data.rc_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    // Process global position data
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);
                    
                    // Store global position data
                    mavlink_data.lat = pos.lat;
                    mavlink_data.lon = pos.lon;
                    mavlink_data.alt = pos.alt;
                    mavlink_data.relative_alt = pos.relative_alt;
                    mavlink_data.vx = pos.vx;
                    mavlink_data.vy = pos.vy;
                    mavlink_data.vz = pos.vz;
                    mavlink_data.hdg = pos.hdg;
                    mavlink_data.global_position_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_RAW_IMU: {
                    // Process raw IMU data
                    mavlink_raw_imu_t imu;
                    mavlink_msg_raw_imu_decode(&msg, &imu);
                    
                    // Store raw IMU data
                    mavlink_data.xacc = imu.xacc;
                    mavlink_data.yacc = imu.yacc;
                    mavlink_data.zacc = imu.zacc;
                    mavlink_data.xgyro = imu.xgyro;
                    mavlink_data.ygyro = imu.ygyro;
                    mavlink_data.zgyro = imu.zgyro;
                    mavlink_data.xmag = imu.xmag;
                    mavlink_data.ymag = imu.ymag;
                    mavlink_data.zmag = imu.zmag;
                    mavlink_data.raw_imu_valid = true;
                    
                    break;
                }
                
                case MAVLINK_MSG_ID_SCALED_PRESSURE: {
                    // Process barometer data
                    mavlink_scaled_pressure_t press;
                    mavlink_msg_scaled_pressure_decode(&msg, &press);
                    
                    // Store barometer data
                    mavlink_data.press_abs = press.press_abs;
                    mavlink_data.press_diff = press.press_diff;
                    mavlink_data.temperature = press.temperature;
                    mavlink_data.scaled_pressure_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_HOME_POSITION: {
                    // Process home position data
                    mavlink_home_position_t home;
                    mavlink_msg_home_position_decode(&msg, &home);
                    
                    // Store home position data
                    mavlink_data.home_lat = home.latitude;
                    mavlink_data.home_lon = home.longitude;
                    mavlink_data.home_alt = home.altitude;
                    mavlink_data.home_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_HIGHRES_IMU: {
                    // Process high-resolution IMU data
                    mavlink_highres_imu_t imu;
                    mavlink_msg_highres_imu_decode(&msg, &imu);
                    
                    // Store high-res IMU data
                    mavlink_data.highres_xacc = imu.xacc;  // m/s²
                    mavlink_data.highres_yacc = imu.yacc;  // m/s²
                    mavlink_data.highres_zacc = imu.zacc;  // m/s²
                    mavlink_data.highres_xgyro = imu.xgyro;  // rad/s
                    mavlink_data.highres_ygyro = imu.ygyro;  // rad/s
                    mavlink_data.highres_zgyro = imu.zgyro;  // rad/s
                    mavlink_data.highres_xmag = imu.xmag;  // gauss
                    mavlink_data.highres_ymag = imu.ymag;  // gauss
                    mavlink_data.highres_zmag = imu.zmag;  // gauss
                    mavlink_data.highres_abs_pressure = imu.abs_pressure;  // hPa
                    mavlink_data.highres_diff_pressure = imu.diff_pressure;  // hPa
                    mavlink_data.highres_temperature = imu.temperature;  // degrees Celsius
                    mavlink_data.pressure_alt = imu.pressure_alt;  // meters
                    mavlink_data.highres_imu_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_SYSTEM_TIME: {
                    // Process system time message
                    mavlink_system_time_t system_time;
                    mavlink_msg_system_time_decode(&msg, &system_time);
                    
                    // Store system time data
                    mavlink_data.time_unix_usec = system_time.time_unix_usec;
                    mavlink_data.time_boot_ms = system_time.time_boot_ms;
                    mavlink_data.system_time_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_LOGGING_DATA_ACKED: {
                    // Process logging data (not storing the actual log data, just acknowledging receipt)
                    mavlink_logging_data_acked_t log_data;
                    mavlink_msg_logging_data_acked_decode(&msg, &log_data);
                    
                    // Send acknowledgment
                    mavlink_message_t ack_msg;
                    mavlink_msg_logging_ack_pack(
                        systemId,
                        componentId,
                        &ack_msg,
                        targetSystem,
                        targetComponent,
                        log_data.sequence
                    );
                    uint16_t len = mavlink_msg_to_send_buffer(buffer, &ack_msg);
                    Serial2.write(buffer, len);
                    break;
                }
                
                case MAVLINK_MSG_ID_LOGGING_DATA: {
                    // Process logging data (not storing the actual log data here)
                    // This just demonstrates receipt of logging data
                    mavlink_data.logging_active = true;
                    mavlink_data.logging_status_valid = true;
                    break;
                }
                
                case MAVLINK_MSG_ID_STATUSTEXT: {
                    mavlink_statustext_t statustext;
                    mavlink_msg_statustext_decode(&msg, &statustext);
                    
                    // Convert the status text to a null-terminated string
                    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
                    memcpy(text, statustext.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
                    text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = '\0';
                    
                    // Check if this is a logging status message
                    if (strstr(text, "log") != NULL || strstr(text, "Log") != NULL) {
                        // Serial.print("Logger message: ");
                        // Serial.println(text);
                        
                        // Parse status information from the text
                        if (strstr(text, "start") != NULL || strstr(text, "START") != NULL) {
                            mavlink_data.logging_active = true;
                            mavlink_data.logging_status_valid = true;
                        } 
                        else if (strstr(text, "stop") != NULL || strstr(text, "STOP") != NULL) {
                            mavlink_data.logging_active = false;
                            mavlink_data.logging_status_valid = true;
                        }
                    }
                    break;
                }
                
                case MAVLINK_MSG_ID_PARAM_VALUE: {
                    mavlink_param_value_t param_value;
                    mavlink_msg_param_value_decode(&msg, &param_value);
                    
                    // Null-terminate the parameter id string
                    char param_id[17];
                    memcpy(param_id, param_value.param_id, 16);
                    param_id[16] = '\0';
                    
                    // Check for logging-related parameters
                    if (strncmp(param_id, "LOG_", 4) == 0) {
                        // Found a logging parameter
                        // Serial.print("Logging parameter: ");
                        // Serial.print(param_id);
                        // Serial.print(" = ");
                        // Serial.println(param_value.param_value);
                        
                        // Update our logging status information
                        if (strcmp(param_id, "LOG_BACKEND_TYPE") == 0) {
                            // A value > 0 typically means logging is enabled
                            mavlink_data.logging_active = (param_value.param_value > 0);
                            mavlink_data.logging_status_valid = true;
                        }
                        else if (strcmp(param_id, "LOG_FILE_BUFSIZE") == 0) {
                            // This gives us an idea of the buffer size which might help estimate write rate
                            mavlink_data.logging_write_rate = param_value.param_value * 512; // Rough estimate
                            mavlink_data.logging_status_valid = true;
                        }
                    }
                    break;
                }
                
                case MAVLINK_MSG_ID_COMMAND_ACK: {
                    mavlink_command_ack_t ack;
                    mavlink_msg_command_ack_decode(&msg, &ack);
                    
                    // Check if this is an acknowledgment for one of our logging commands
                    if (ack.command == MAV_CMD_LOGGING_START) {
                        // Serial.print("Logging start command acknowledged with result: ");
                        // Serial.println(ack.result);
                        mavlink_data.logging_active = (ack.result == MAV_RESULT_ACCEPTED);
                    }
                    else if (ack.command == MAV_CMD_LOGGING_STOP) {
                        // Serial.print("Logging stop command acknowledged with result: ");
                        // Serial.println(ack.result);
                        mavlink_data.logging_active = !(ack.result == MAV_RESULT_ACCEPTED);
                    }
                    // Check what command this is acknowledging
                    if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                        if (ack.result == MAV_RESULT_ACCEPTED) {
                            // Serial.println("Arm/disarm command accepted!");
                            if (ack.progress > 0) {
                                mavlink_data.logging_active = true;
                                // Serial.println("Vehicle armed - logging should now be active");
                            } else {
                                mavlink_data.logging_active = false;
                                // Serial.println("Vehicle disarmed - logging should stop");
                            }
                        } else {
                            // Serial.print("Arm/disarm command rejected with result: ");
                            // Serial.println(ack.result);
                            // Serial.println("Possible reasons: safety check failed, not in correct mode, or other pre-arm checks failed");
                        }
                    }
                    else if (ack.command == MAV_CMD_LOGGING_START) {
                        // ... existing logging ACK handling ...
                    }
                    // ... other existing ACK handling ...
                    break;
                }
                
                case MAVLINK_MSG_ID_VIBRATION: {
                    // Process vibration data
                    mavlink_vibration_t vibration;
                    mavlink_msg_vibration_decode(&msg, &vibration);
                    
                    // Store vibration data
                    mavlink_data.vibration_x = vibration.vibration_x;
                    mavlink_data.vibration_y = vibration.vibration_y;
                    mavlink_data.vibration_z = vibration.vibration_z;
                    mavlink_data.clipping_x = vibration.clipping_0;
                    mavlink_data.clipping_y = vibration.clipping_1;
                    mavlink_data.clipping_z = vibration.clipping_2;
                    mavlink_data.vibration_valid = true;
                    break;
                }
                
                default:
                    // Other message types can be handled here
                    break;
            }
        }
    }
    
    return messageReceived;
}

void MavlinkDecoder::sendHeartbeat() {
    // Initialize the message buffer
    mavlink_message_t msg;
    
    // Pack a heartbeat message
    mavlink_msg_heartbeat_pack(
        systemId,          // System ID
        componentId,       // Component ID
        &msg,              // Message buffer
        MAV_TYPE_GCS,      // Type (ground control station)
        MAV_AUTOPILOT_INVALID, // Autopilot type 
        0,                 // Base mode
        0,                 // Custom mode
        MAV_STATE_ACTIVE   // System status
    );
    
    // Convert the message to a byte buffer
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Send the message
    Serial2.write(buffer, len);
}

void MavlinkDecoder::requestAllDataStreams(uint8_t streamRate) {
    // Define the streams we want to request
    const uint8_t streams[] = {
        MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1,     // Attitude
        MAV_DATA_STREAM_EXTRA2,     // VFR_HUD
        MAV_DATA_STREAM_EXTRA3,     // Parameters
        MAV_DATA_STREAM_ALL         // All data streams (last to override previous settings)
    };

    const int numStreams = sizeof(streams) / sizeof(streams[0]);
    mavlink_message_t msg;
    uint16_t len;

    // Request each stream
    for (int i = 0; i < numStreams; i++) {
        mavlink_msg_request_data_stream_pack(
            systemId,           // System ID (sender)
            componentId,        // Component ID (sender)
            &msg,              // Message buffer
            targetSystem,      // Target system
            targetComponent,   // Target component
            streams[i],        // Stream ID
            streamRate,        // Rate in Hz
            1                  // Start/Stop (1 = start)
        );

        // Convert the message to a byte buffer
        len = mavlink_msg_to_send_buffer(buffer, &msg);
        
        // Send the message
        Serial2.write(buffer, len);
        
        // Short delay to avoid flooding the autopilot
        // delay(10);
    }
    
    // For newer ArduPilot/PX4 firmwares, request high-res IMU specifically
    // by setting the appropriate parameters
    requestHighResIMU();
    
    // Serial.println("Requested all MAVLink data streams");
    
    // Also request streams by specific message ID
    requestSpecificStreams();
}

// Add this new method to specifically request HIGHRES_IMU messages
void MavlinkDecoder::requestHighResIMU() {
    // Request the HIGHRES_IMU message directly by setting the appropriate parameters
    mavlink_message_t msg;
    uint16_t len;
    
    // Set parameter to enable high-rate IMU (for ArduPilot)
    // Parameter name depends on firmware - these are common ones
    const char* paramNames[] = {"INS_FAST_SAMPLE", "IMU_HIRES_RATE"};
    
    for (int i = 0; i < 2; i++) {
        // Try to set parameter to enable high-rate IMU
        mavlink_msg_param_set_pack(
            systemId,           // System ID (sender)
            componentId,        // Component ID (sender)
            &msg,               // Message buffer
            targetSystem,       // Target system
            targetComponent,    // Target component
            paramNames[i],      // Parameter name
            1.0f,               // Parameter value (enable)
            MAV_PARAM_TYPE_REAL32 // Parameter type
        );
        
        // Convert the message to a byte buffer
        len = mavlink_msg_to_send_buffer(buffer, &msg);
        
        // Send the message
        Serial2.write(buffer, len);
        // delay(20);
    }
    
    // For PX4, directly request HIGHRES_IMU at a higher rate
    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        MAV_CMD_SET_MESSAGE_INTERVAL,  // Command to set message interval
        0,                            // Confirmation
        MAVLINK_MSG_ID_HIGHRES_IMU,   // Message ID for HIGHRES_IMU
        20000,                        // Interval in microseconds (50Hz)
        0, 0, 0, 0, 0                 // Empty parameters
    );
    
    // Convert the message to a byte buffer
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Send the message
    Serial2.write(buffer, len);
    
    // Serial.println("Requested HIGHRES_IMU data stream specifically");
}

void MavlinkDecoder::requestSpecificStreams() {
    // Define a structure to hold message ID and rate
    struct StreamInfo {
        uint16_t msgId;
        uint8_t rate;
    };
    
    // Array of message IDs and rates based on the console commands
    // Format: {MAVLink Message ID, Rate in Hz}
    const StreamInfo streams[] = {
        {MAVLINK_MSG_ID_ALTITUDE, 5},
        {MAVLINK_MSG_ID_ATTITUDE, 5},
        {MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 5},
        {MAVLINK_MSG_ID_ATTITUDE_TARGET, 5},
        {MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE, 0},  // 0 means disabled
        {MAVLINK_MSG_ID_ESTIMATOR_STATUS, 1},
        {MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5},
        {MAVLINK_MSG_ID_HEARTBEAT, 1},
        {MAVLINK_MSG_ID_HIGHRES_IMU, 5},
        {MAVLINK_MSG_ID_LINK_NODE_STATUS, 1},
        {MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5},
        {MAVLINK_MSG_ID_MISSION_CURRENT, 1},
        {MAVLINK_MSG_ID_ODOMETRY, 5},
        {MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION, 1},
        {MAVLINK_MSG_ID_PING, 1},
        {MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10},
        {MAVLINK_MSG_ID_SCALED_PRESSURE, 1},
        {MAVLINK_MSG_ID_SYSTEM_TIME, 1},
        {MAVLINK_MSG_ID_SYS_STATUS, 4},
        {MAVLINK_MSG_ID_TIMESYNC, 10},
        {MAVLINK_MSG_ID_GPS_RAW_INT, 5},
        {MAVLINK_MSG_ID_HOME_POSITION, 1},
        {MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5},
        {MAVLINK_MSG_ID_VIBRATION, 5}    // Request vibration data at 5Hz
    };
    
    const int numStreams = sizeof(streams) / sizeof(streams[0]);
    mavlink_message_t msg;
    uint16_t len;
    
    // Serial.println("Requesting specific MAVLink data streams...");
    
    // Send a command to set the message interval for each stream
    for (int i = 0; i < numStreams; i++) {
        // Skip disabled streams (rate = 0)
        if (streams[i].rate == 0) continue;
        
        // Calculate interval in microseconds (1,000,000 μs / rate Hz)
        uint32_t interval_us = streams[i].rate > 0 ? (1000000 / streams[i].rate) : 0;
        
        // Pack a COMMAND_LONG message to set the message interval
        mavlink_msg_command_long_pack(
            systemId,                     // System ID (sender)
            componentId,                  // Component ID (sender)
            &msg,                         // Message buffer
            targetSystem,                 // Target system
            targetComponent,              // Target component (usually 0 = autopilot)
            MAV_CMD_SET_MESSAGE_INTERVAL, // Command ID
            0,                            // Confirmation (first transmission)
            streams[i].msgId,             // Parameter 1: Message ID
            interval_us,                  // Parameter 2: Interval in microseconds
            0, 0, 0, 0, 0                 // Parameters 3-7 (unused)
        );
        
        // Convert the message to a byte buffer
        len = mavlink_msg_to_send_buffer(buffer, &msg);
        
        // Send the message
        Serial2.write(buffer, len);
        
        // Short delay to avoid flooding the autopilot
        // delay(10);
        
        // Debug output 
        // Serial.print("Requested stream ID ");
        // Serial.print(streams[i].msgId);
        // Serial.print(" at ");
        // Serial.print(streams[i].rate);
        // Serial.println(" Hz");
    }
    
    // Serial.println("All stream requests sent!");
}

bool MavlinkDecoder::getAttitude(float &roll, float &pitch, float &yaw) {
    if (mavlink_data.attitude_valid) {
        roll = mavlink_data.roll;
        pitch = mavlink_data.pitch;
        yaw = mavlink_data.yaw;
        return true;
    }
    return false;
}

bool MavlinkDecoder::getGPSInfo(int32_t &lat, int32_t &lon, int32_t &alt, uint8_t &satellites) {
    if (mavlink_data.gps_valid) {
        lat = mavlink_data.lat;
        lon = mavlink_data.lon;
        alt = mavlink_data.alt;
        satellites = mavlink_data.satellites;
        return true;
    }
    return false;
}

bool MavlinkDecoder::getBatteryInfo(float &voltage, float &current, int8_t &remaining) {
    if (mavlink_data.battery_valid) {
        voltage = mavlink_data.voltage;
        current = mavlink_data.current;
        remaining = mavlink_data.remaining;
        return true;
    }
    return false;
}

bool MavlinkDecoder::getVfrHudData(float &airspeed, float &groundspeed, float &heading, 
                                  float &throttle, float &alt, float &climb) {
    if (mavlink_data.vfr_valid) {
        airspeed = mavlink_data.airspeed;
        groundspeed = mavlink_data.groundspeed;
        heading = mavlink_data.heading;
        throttle = mavlink_data.throttle;
        alt = mavlink_data.alt_vfr;
        climb = mavlink_data.climb;
        return true;
    }
    return false;
}

bool MavlinkDecoder::getRcChannels(uint16_t* channels, uint8_t &chancount) {
    if (mavlink_data.rc_valid) {
        chancount = mavlink_data.chancount;
        for (int i = 0; i < chancount && i < 18; i++) {
            channels[i] = mavlink_data.rc_channels[i];
        }
        return true;
    }
    return false;
}

bool MavlinkDecoder::getHighResImu(float &xacc, float &yacc, float &zacc, 
                                  float &xgyro, float &ygyro, float &zgyro,
                                  float &xmag, float &ymag, float &zmag,
                                  float &abs_pressure, float &diff_pressure, float &temperature, float &pressure_alt) {
    if (mavlink_data.highres_imu_valid) {
        xacc = mavlink_data.highres_xacc;
        yacc = mavlink_data.highres_yacc;
        zacc = mavlink_data.highres_zacc;
        xgyro = mavlink_data.highres_xgyro;
        ygyro = mavlink_data.highres_ygyro;
        zgyro = mavlink_data.highres_zgyro;
        xmag = mavlink_data.highres_xmag;
        ymag = mavlink_data.highres_ymag;
        zmag = mavlink_data.highres_zmag;
        abs_pressure = mavlink_data.highres_abs_pressure;
        diff_pressure = mavlink_data.highres_diff_pressure;
        temperature = mavlink_data.highres_temperature;
        pressure_alt = mavlink_data.pressure_alt;
        return true;
    }
    return false;
}

bool MavlinkDecoder::getSystemTime(uint64_t &unix_time_usec, uint32_t &boot_time_ms) {
    if (mavlink_data.system_time_valid) {
        unix_time_usec = mavlink_data.time_unix_usec;
        boot_time_ms = mavlink_data.time_boot_ms;
        return true;
    }
    return false;
}

bool MavlinkDecoder::startLogging() {
    mavlink_message_t msg;
    
    // First method: Standard MAVLink command for logging
    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        MAV_CMD_LOGGING_START,  // Command ID for starting logging
        0,                      // Confirmation number
        0,                      // Parameter 1: Format (0=native)
        0, 0, 0, 0, 0, 0        // Parameters (unused)
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    // delay(50);  // Wait for processing
    
    // Second method: For ArduPilot, set LOG_BACKEND_TYPE parameter to enable file backend
    mavlink_msg_param_set_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        "LOG_BACKEND_TYPE",    // Parameter name
        2,                     // Value (2 = enable file backend)
        MAV_PARAM_TYPE_INT8    // Parameter type
    );
    
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    // delay(50);  // Wait for processing
    
    // Third method: PX4 specific - use a command that the firmware supports
    // Replace custom command with a MAVLink parameter set
    mavlink_msg_param_set_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        "SDLOG_MODE",         // PX4 logging mode parameter
        1,                    // Value (1 = enabled)
        MAV_PARAM_TYPE_INT8   // Parameter type
    );
    
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    
    // Update our internal state intention
    mavlink_data.logging_active = true;
    
    // Request parameters to update status
    requestLogParameters();
    
    // Serial.println("Logging start commands sent (multiple methods)");
    return true;
}

bool MavlinkDecoder::stopLogging() {
    mavlink_message_t msg;
    
    // First method: Standard MAVLink command
    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        MAV_CMD_LOGGING_STOP,   // Command to stop logging
        0,                      // Confirmation
        0, 0, 0, 0, 0, 0, 0     // Parameters (unused)
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    // delay(50);  // Wait for processing
    
    // Second method: For ArduPilot, set LOG_BACKEND_TYPE parameter to disable
    mavlink_msg_param_set_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        "LOG_BACKEND_TYPE",    // Parameter name
        0,                     // Value (0 = disable)
        MAV_PARAM_TYPE_INT8    // Parameter type
    );
    
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    // delay(50);  // Wait for processing
    
    // Third method: PX4 specific - use a command that the firmware supports
    // Replace custom command with a MAVLink parameter set
    mavlink_msg_param_set_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        "SDLOG_MODE",         // PX4 logging mode parameter
        0,                    // Value (0 = disabled)
        MAV_PARAM_TYPE_INT8   // Parameter type
    );
    
    len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    
    // Update our internal state intention
    mavlink_data.logging_active = false;
    
    // Serial.println("Logging stop commands sent (multiple methods)");
    return true;
}

bool MavlinkDecoder::getLoggingStatus() {
    // Request parameters that indicate logging status
    requestLogParameters();
    
    // For PX4, request specific logging parameters
    mavlink_message_t msg;
    
    mavlink_msg_param_request_read_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        "SDLOG_MODE",  // Parameter name
        -1            // Parameter index (-1 = use name)
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    
    // Return current known status
    return mavlink_data.logging_active;
}

bool MavlinkDecoder::getLoggingStats(uint32_t &write_rate, uint32_t &space_left) {
    // Send status command if we haven't recently
    static unsigned long last_status_request = 0;
    if (millis() - last_status_request > 5000) {
        getLoggingStatus();
        last_status_request = millis();
    }
    
    // Return the stats we have (may be from previous status responses)
    if (mavlink_data.logging_status_valid) {
        write_rate = mavlink_data.logging_write_rate;
        space_left = mavlink_data.logging_space_left;
        return true;
    }
    return false;
}

// General purpose function to send shell commands to Pixhawk
void MavlinkDecoder::sendShellCommand(const char* command) {
    // For most Pixhawk firmwares, there isn't a direct MAVLink message for shell commands
    // Instead, we'll need to use custom command or MAV_CMD_DO_COMMAND_LONG with appropriate parameters
    
    mavlink_message_t msg;
    bool commandSent = false;
    
    // Try to determine what kind of command we're sending
    if (strncmp(command, "logger", 6) == 0) {
        // Special handling for logger commands
        if (strcmp(command, "logger on") == 0) {
            // Start logging - call our dedicated function instead
            startLogging();
            return;
        }
        else if (strcmp(command, "logger off") == 0) {
            // Stop logging - call our dedicated function instead
            stopLogging();
            return;
        }
        else if (strcmp(command, "logger status") == 0) {
            // Get logging status - use our dedicated function
            getLoggingStatus();
            return;
        }
    }
    
    // For generic shell commands, we'll use a different approach
    // Unfortunately, we can't directly send arbitrary shell commands via standard MAVLink
    // We'll need to implement specific handlers for each command type we want to support
    
    // Let the user know that this command might not be supported
    // Serial.print("Shell command not directly supported: ");
    // Serial.println(command);
    // Serial.println("Using specific command handlers instead.");
    
    // Give the Pixhawk time to process the command
    // delay(50);
}

// Helper method to request logging parameters
void MavlinkDecoder::requestLogParameters() {
    // Request common parameters related to logging
    const char* params[] = {
        "LOG_BACKEND_TYPE", 
        "LOG_FILE_BUFSIZE", 
        "LOG_DISARMED",
        "SDLOG_MODE",       // PX4 specific
        "SDLOG_PROFILE"     // PX4 specific
    };
    
    mavlink_message_t msg;
    
    for (int i = 0; i < sizeof(params)/sizeof(params[0]); i++) {
        mavlink_msg_param_request_read_pack(
            systemId,
            componentId,
            &msg,
            targetSystem,
            targetComponent,
            params[i],  // Parameter name
            -1          // Parameter index (-1 = use name)
        );
        
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        Serial2.write(buffer, len);
        // delay(10);
    }
    
    // Serial.println("Requested logging parameters");
}

bool MavlinkDecoder::armVehicle(bool arm) {
    mavlink_message_t msg;
    
    // Pack a command to arm/disarm the vehicle
    mavlink_msg_command_long_pack(
        systemId,
        componentId,
        &msg,
        targetSystem,
        targetComponent,
        MAV_CMD_COMPONENT_ARM_DISARM,  // Command ID for arming/disarming
        0,                             // Confirmation number
        arm ? 1.0f : 0.0f,            // Parameter 1: 1 = arm, 0 = disarm
        0,                            // Parameter 2: Force (0 = normal)
        0, 0, 0, 0, 0                 // Parameters 3-7 (unused)
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    
    // Serial.println(arm ? "Arm command sent - this should trigger logging" : "Disarm command sent");
    
    // The result of this command will be received in a COMMAND_ACK message
    return true;
}

bool MavlinkDecoder::getVibrationData(float &vibe_x, float &vibe_y, float &vibe_z,
                                     uint32_t &clip_x, uint32_t &clip_y, uint32_t &clip_z) {
    if (mavlink_data.vibration_valid) {
        vibe_x = mavlink_data.vibration_x;
        vibe_y = mavlink_data.vibration_y;
        vibe_z = mavlink_data.vibration_z;
        clip_x = mavlink_data.clipping_x;
        clip_y = mavlink_data.clipping_y;
        clip_z = mavlink_data.clipping_z;
        return true;
    }
    return false;
}

bool MavlinkDecoder::getGpsTime(uint64_t &gps_time_usec, uint8_t &fix_type) {
    if (mavlink_data.gps_time_valid) {
        gps_time_usec = mavlink_data.gps_time_usec;
        fix_type = mavlink_data.gps_fix_type;
        Serial.println(mavlink_data.gps_time_usec);
        return true;
    }
    return false;
}

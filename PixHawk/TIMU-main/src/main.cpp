#include <Arduino.h>
#include <common/mavlink.h>
#include <Streaming.h>
#include <SPI.h>
#include <SD.h>
#include <TeensyThreads.h>

#define VERSION "0.0.1"

#include "HighSpeedLogger.h"
#include "linker.h"
#include "autopilot_interface.h"

HighSpeedLogger logger(new SDClass(SD));
Linker linker(logger);
Autopilot_Interface pixhawk(&linker);

// For timing the serial output
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 21;
// const unsigned long PRINT_INTERVAL = 2000;

// Add at the top with other globals
unsigned long lastStreamRequestTime = 0;
const unsigned long STREAM_REQUEST_INTERVAL = 2000;

// Add these globals at the top with other variables
unsigned long lastAttitudeTime = 0;
unsigned long lastIMUTime = 0;
float prev_values[12] = {0}; // Store previous valid values

void printAllMavlinkData(const Mavlink_Messages& messages) {
  // System info
  // SerialUSB << "=== MAVLINK DEBUG INFO ===" << endl;
  // SerialUSB << "System ID: " << messages.sysid << ", Component ID: " << messages.compid << endl;
  
  // // Heartbeat
  // SerialUSB << "--- HEARTBEAT ---" << endl;
  // SerialUSB << "Type: " << (int)messages.heartbeat.type << ", Autopilot: " << (int)messages.heartbeat.autopilot 
  //           << ", Base mode: " << (int)messages.heartbeat.base_mode << ", Custom mode: " << messages.heartbeat.custom_mode 
  //           << ", System status: " << (int)messages.heartbeat.system_status << endl;
  
  // // System Status
  // SerialUSB << "--- SYSTEM STATUS ---" << endl;
  // SerialUSB << "Voltage (battery): " << messages.sys_status.voltage_battery << " mV" 
  //           << ", Current: " << messages.sys_status.current_battery << " cA"
  //           << ", Remaining: " << (int)messages.sys_status.battery_remaining << "%" << endl;
  
  // // Battery Status
  // SerialUSB << "--- BATTERY STATUS ---" << endl;
  // SerialUSB << "Temperature: " << messages.battery_status.temperature << " C"
  //           << ", Current consumed: " << messages.battery_status.current_consumed << " mAh" << endl;
  
  // // Radio Status
  // SerialUSB << "--- RADIO STATUS ---" << endl;
  // SerialUSB << "RSSI: " << messages.radio_status.rssi 
  //           << ", Remote RSSI: " << messages.radio_status.remrssi
  //           << ", Noise: " << messages.radio_status.noise << endl;
  
  // // Position (Local)
  // SerialUSB << "--- LOCAL POSITION ---" << endl;
  // SerialUSB << "X: " << messages.local_position_ned.x 
  //           << ", Y: " << messages.local_position_ned.y
  //           << ", Z: " << messages.local_position_ned.z << " m" << endl;
  
  // // Position (Global)
  SerialUSB << "--- GLOBAL POSITION ---" << endl;
  SerialUSB << "Lat: " << messages.global_position_int.lat / 10000000.0 
            << ", Lon: " << messages.global_position_int.lon / 10000000.0
            << ", Alt: " << messages.global_position_int.alt / 1000.0 << " m" << endl;
  
  // High-res IMU
  SerialUSB << "--- HIGH-RES IMU ---" << endl;
  SerialUSB << "Acc (x,y,z): " << messages.highres_imu.xacc << ", " 
            << messages.highres_imu.yacc << ", " 
            << messages.highres_imu.zacc << " m/s^2" << endl;
  SerialUSB << "Gyro (x,y,z): " << messages.highres_imu.xgyro << ", " 
            << messages.highres_imu.ygyro << ", " 
            << messages.highres_imu.zgyro << " rad/s" << endl;
  SerialUSB << "Mag (x,y,z): " << messages.highres_imu.xmag << ", " 
            << messages.highres_imu.ymag << ", " 
            << messages.highres_imu.zmag << " gauss" << endl;
  SerialUSB << "Pressure: " << messages.highres_imu.abs_pressure << " hPa" << endl;
  SerialUSB << "Temperature: " << messages.highres_imu.temperature << " C" << endl;
  SerialUSB << "IMU fields_updated bitmap: " << messages.highres_imu.fields_updated << endl;
  SerialUSB << "Pressure updated: " << ((messages.highres_imu.fields_updated & 512) ? "YES" : "NO") << endl;
  SerialUSB << "Temperature updated: " << ((messages.highres_imu.fields_updated & 4096) ? "YES" : "NO") << endl;
  
  // // Attitude
  // SerialUSB << "--- ATTITUDE ---" << endl;
  // SerialUSB << "Roll: " << degrees(messages.attitude.roll) 
  //           << ", Pitch: " << degrees(messages.attitude.pitch)
  //           << ", Yaw: " << degrees(messages.attitude.yaw) << " deg" << endl;
  
  // // Attitude Quaternion
  // SerialUSB << "--- ATTITUDE QUATERNION ---" << endl;
  // SerialUSB << "q1: " << messages.attitude_quaternion.q1 
  //           << ", q2: " << messages.attitude_quaternion.q2
  //           << ", q3: " << messages.attitude_quaternion.q3
  //           << ", q4: " << messages.attitude_quaternion.q4 << endl;
  
  // // Estimator Status
  // SerialUSB << "--- ESTIMATOR STATUS ---" << endl;
  // SerialUSB << "Flags: " << messages.estimator_status.flags 
  //           << ", EKF error: " << messages.estimator_status.pos_horiz_accuracy << endl;
  
  // // Vibration
  // SerialUSB << "--- VIBRATION ---" << endl;
  // SerialUSB << "X: " << messages.vibration.vibration_x 
  //           << ", Y: " << messages.vibration.vibration_y
  //           << ", Z: " << messages.vibration.vibration_z << endl;
  
  // // Altitude
  // SerialUSB << "--- ALTITUDE ---" << endl;
  // SerialUSB << "Altitude (amsl): " << messages.altitude.altitude_amsl
  //           << ", Relative: " << messages.altitude.altitude_relative << " m" << endl;
  
  // GPS Data
  // SerialUSB << "--- GPS DATA ---" << endl;
  // SerialUSB << "Fix type: " << (int)messages.gps_raw.fix_type
  //           << ", Satellites: " << (int)messages.gps_raw.satellites_visible
  //           << ", HDOP: " << messages.gps_raw.eph/100.0 << endl;
  
  SerialUSB << "=====================" << endl << endl;
}

void testPX4IMU(const Mavlink_Messages& messages) {
  // Check if we've received fresh IMU data (using fields_updated)
  bool fresh_imu = (messages.highres_imu.fields_updated != 0);
  bool fresh_attitude = (messages.attitude.time_boot_ms > lastAttitudeTime);
  
  if (fresh_attitude) {
    lastAttitudeTime = messages.attitude.time_boot_ms;
    prev_values[0] = messages.attitude.roll;
    prev_values[1] = messages.attitude.pitch;
    prev_values[2] = messages.attitude.yaw;
    prev_values[3] = messages.attitude.rollspeed;
    prev_values[4] = messages.attitude.pitchspeed;
    prev_values[5] = messages.attitude.yawspeed;
  }
  
  if (fresh_imu) {
    lastIMUTime = millis();
    prev_values[6] = messages.highres_imu.xacc;
    prev_values[7] = messages.highres_imu.yacc;
    prev_values[8] = messages.highres_imu.zacc;
    prev_values[9] = messages.highres_imu.xgyro;
    prev_values[10] = messages.highres_imu.ygyro;
    prev_values[11] = messages.highres_imu.zgyro;
  }
  
  // Print previous valid values if not fresh or current values if fresh
  SerialUSB << (fresh_attitude ? messages.attitude.roll : prev_values[0]) << ","
            << (fresh_attitude ? messages.attitude.pitch : prev_values[1]) << ","
            << (fresh_attitude ? messages.attitude.yaw : prev_values[2]) << ","
            << (fresh_attitude ? messages.attitude.rollspeed : prev_values[3]) << ","
            << (fresh_attitude ? messages.attitude.pitchspeed : prev_values[4]) << ","
            << (fresh_attitude ? messages.attitude.yawspeed : prev_values[5]) << ","
            << (fresh_imu ? messages.highres_imu.xacc : prev_values[6]) << ","
            << (fresh_imu ? messages.highres_imu.yacc : prev_values[7]) << ","
            << (fresh_imu ? messages.highres_imu.zacc : prev_values[8]) << ","
            << (fresh_imu ? messages.highres_imu.xgyro : prev_values[9]) << ","
            << (fresh_imu ? messages.highres_imu.ygyro : prev_values[10]) << ","
            << (fresh_imu ? messages.highres_imu.zgyro : prev_values[11]) << endl;
}

void PXthread(){
  while(1){
    pixhawk.read_messages();
    Mavlink_Messages messages = pixhawk.current_messages;

    unsigned long currentTime = millis();

  // Only print data periodically to avoid flooding the serial monitor
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    // printAllMavlinkData(messages);
    testPX4IMU(messages);
  }
  }
}

void setup() {
  Serial2.begin(921600);
  SerialUSB.begin(921600);

  delay(1000);
  
  SerialUSB << "TIMU - " << VERSION << " initialized" << endl;
  
  // Request HIGHRES_IMU data at 50Hz
  // pixhawk.request_data_stream(MAV_DATA_STREAM_RAW_SENSORS, 50);
  // pixhawk.request_data_stream(MAV_DATA_STREAM_EXTENDED_STATUS, 10);
  // pixhawk.request_data_stream(MAV_DATA_STREAM_POSITION, 10);
  // pixhawk.request_data_stream(MAV_DATA_STREAM_EXTRA1, 20); // Attitude data

  pixhawk.request_data_stream(MAV_DATA_STREAM_ALL, 50); // Request all data streams at 50Hz
  // threads.addThread(PXthread);
}

void loop() {
  // Read messages from Pixhawk
  pixhawk.read_messages();
  
  // Get the latest message data from the autopilot interface
  Mavlink_Messages messages = pixhawk.current_messages;
  
  // Periodically request data streams
  unsigned long currentTime = millis();

  // Request data stream every 2 seconds
  // if (currentTime - lastStreamRequestTime >= STREAM_REQUEST_INTERVAL) {
  //   lastStreamRequestTime = currentTime;
  //   pixhawk.request_data_stream(MAV_DATA_STREAM_ALL, 50); // Request all data streams at 50Hz
  // }

  // Only print data periodically to avoid flooding the serial monitor
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;
    
    // Print all message data for debugging
    // printAllMavlinkData(messages);
    testPX4IMU(messages);
  }
}
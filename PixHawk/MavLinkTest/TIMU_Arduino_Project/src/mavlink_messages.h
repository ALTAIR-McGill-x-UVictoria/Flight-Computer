#ifndef MAVLINK_MESSAGES_H
#define MAVLINK_MESSAGES_H

#include <common/mavlink.h>

struct Time_Stamps {
    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;
    uint64_t attitude_quaternion;
    uint64_t estimator_status;
    uint64_t odometry;
    uint64_t vibration;
    uint64_t altitude;
    uint64_t gps_rtk;
    uint64_t gps_global_origin;
    uint64_t gps_raw;
    uint64_t gps_status;
};

struct Mavlink_Messages {
    mavlink_heartbeat_t heartbeat;
    mavlink_sys_status_t sys_status;
    mavlink_battery_status_t battery_status;
    mavlink_radio_status_t radio_status;
    mavlink_local_position_ned_t local_position_ned;
    mavlink_global_position_int_t global_position_int;
    mavlink_position_target_local_ned_t position_target_local_ned;
    mavlink_position_target_global_int_t position_target_global_int;
    mavlink_highres_imu_t highres_imu;
    mavlink_attitude_t attitude;
    mavlink_attitude_quaternion_t attitude_quaternion;
    mavlink_estimator_status_t estimator_status;
    mavlink_odometry_t odometry;
    mavlink_vibration_t vibration;
    mavlink_altitude_t altitude;
    mavlink_gps_rtk_t gps_rtk;
    mavlink_gps_global_origin_t gps_global_origin;
    mavlink_gps_raw_int_t gps_raw;
    mavlink_gps_status_t gps_status;
    
    Time_Stamps time_stamps;
};

#endif // MAVLINK_MESSAGES_H
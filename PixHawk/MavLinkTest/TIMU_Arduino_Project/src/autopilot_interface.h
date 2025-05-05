#ifndef AUTOPILOT_INTERFACE_H
#define AUTOPILOT_INTERFACE_H

#include <Arduino.h>
#include <common/mavlink.h>
#include "linker.h"

class Autopilot_Interface {
public:
    Autopilot_Interface(Linker *linker);
    ~Autopilot_Interface();

    void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
    void read_messages();
    int write_message(mavlink_message_t message);

private:
    Linker *linker;
    uint8_t write_count;
    uint8_t reading_status;
    uint8_t writing_status;
    uint8_t control_status;
    uint8_t system_id;
    uint8_t autopilot_id;
    uint8_t companion_id;

    Mavlink_Messages current_messages;
};

#endif // AUTOPILOT_INTERFACE_H
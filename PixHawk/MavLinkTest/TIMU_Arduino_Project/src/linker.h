#ifndef LINKER_H
#define LINKER_H

#include <Arduino.h>
#include <common/mavlink.h>
#include "HighSpeedLogger.h"

class Linker {
public:
    Linker(HighSpeedLogger __logger);
    ~Linker();
    
    int read_message(mavlink_message_t &message);
    int write_message(const mavlink_message_t &message);
    void stop();

private:
    HighSpeedLogger logger;
    bool is_open;
    int baudrate;
    mavlink_status_t lastStatus;
    bool debug;
};

#endif // LINKER_H
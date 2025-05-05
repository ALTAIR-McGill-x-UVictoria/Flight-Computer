#ifndef HIGHSPEEDLOGGER_H
#define HIGHSPEEDLOGGER_H

#include <Arduino.h>
#include <common/mavlink.h>
#include <SD.h>

class HighSpeedLogger {
public:
    HighSpeedLogger(SDClass *__card);
    HighSpeedLogger();
    ~HighSpeedLogger();

    int begin(int __cs);
    File open(const char* __name);
    void write(mavlink_message_t __message);
    void close();
    void end();

    bool is_logging;

private:
    SDClass *card;
    File logger;
};

#endif // HIGHSPEEDLOGGER_H
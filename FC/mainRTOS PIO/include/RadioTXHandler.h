#ifndef RADIO_TX_HANDLER_H
#define RADIO_TX_HANDLER_H

#include <RH_RF95.h>
#include "utils.h"

//Radio pin definitions
#define RFM95_RST 23
#define RFM95_CS 10
#define RFM95_INT 22

//LoRa parameters definitions
#define RF95_FREQ 915.0
#define SF 8
#define BW 125000
#define TX_POWER 20


class Radio {
  private:
    RH_RF95* rf95;
    int timeout;
    bool parseReply(const char* reply, int& command, float& argument);
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

  public:
    int lastRSSI;
    int lastSNR;
    int ack;
    Radio();
    ~Radio();
    int setup();
    void radioTx(const char* packet);
    bool radioRx(char* buffer, uint8_t* len);
    void FCradioHandler(const char* packet);

};

#endif

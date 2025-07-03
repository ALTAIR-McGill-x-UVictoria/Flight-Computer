#include "RadioTXHandler.h"
#include "utils.h"  // Include to access buzzer and calibration functions
#include <Arduino.h>
#include <stdlib.h>
#include <SPI.h>
#include <notes.h>


extern altRadioPacket currentAltPacket;
extern radioPacket currentPacket;

Radio::Radio() : timeout(2000) {
    rf95 = new RH_RF95(RFM95_CS, RFM95_INT);
}

Radio::~Radio() {
    delete rf95;
}

int Radio::setup() {

    SPI.begin();
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    // Manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    // Read registers for debug
//   uint8_t modemConfig1 = rf95->spiRead(0x1D); // REG_MODEM_CONFIG1
//   uint8_t modemConfig2 = rf95->spiRead(0x1E); // REG_MODEM_CONFIG2
//   uint8_t modemConfig3 = rf95->spiRead(0x26); // REG_MODEM_CONFIG3

    //   Serial.print("ModemConfig1: 0x"); Serial.println(modemConfig1, HEX);
    // Serial.print("ModemConfig2: 0x"); Serial.println(modemConfig2, HEX);
    // Serial.print("ModemConfig3: 0x"); Serial.println(modemConfig3, HEX);


    if (!rf95->init()) {
        Serial.println("LoRa radio init failed");
        return -1;
    }



    if (!rf95->setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        return -2;
    }

    rf95->setSignalBandwidth(BW);
    rf95->setSpreadingFactor(SF);
    rf95->setTxPower(TX_POWER, false);
    rf95->setCodingRate4(5);  // Set coding rate to 4/5
    rf95->setPayloadCRC(true); // CRITICAL: Match GS setting for CRC
    
    return 1;
}

void Radio::radioTx(const char* packet) {
    rf95->send((uint8_t*)packet, strlen(packet));
    rf95->waitPacketSent();
    
    // CRITICAL: Add this line
    rf95->setModeRx();
}

bool Radio::radioRx(char* buffer, uint8_t* len) {
    // rf95->setModeRx();  // Ensure the radio is in receive mode

    // if (rf95->available()) {
    //     Serial.println("Radio available, reading data...");
    // }

    if (rf95->waitAvailableTimeout(timeout)) {
        if (rf95->recv((uint8_t*)buffer, len)) {
            buffer[*len] = '\0';  // Null terminate the string
            return true;
        }
    }
    return false;
}

void Radio::FCradioHandler(const char* packet) {
    char radiopacket[150];
    char reply[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(reply);
    
    strncpy(radiopacket, packet, sizeof(radiopacket) - 1);
    radiopacket[sizeof(radiopacket) - 1] = '\0';
    
    // Add debug output
    // Serial.print("Sending packet: ");
    // Serial.println(radiopacket);
    
    // Send packet
    radioTx(radiopacket);
    Serial.println("Packet sent");
    
    // Now wait for reply
    if (radioRx(reply, &len)) {
        Serial.print("Reply received: ");
        Serial.println(reply);
        lastRSSI = rf95->lastRssi();
        lastSNR = rf95->lastSNR();
        Serial.println(digitalRead(29));

        int command;
        float argument;
        
        if (parseReply(reply, command, argument)) {
            currentPacket.ack = command;
            currentAltPacket.ack = command;
            
            switch (command) {
            case 1:
                playPingJingle();
                break;
            case 2:
                break;
            case 6:
                // calibrateIMU();
                break;
            case 19:
                // Terminate
              Serial.println("Energizing actuator");
              digitalWrite(29,LOW);
              break;
            case 20:
                // Reset termination
              Serial.println("Resetting actuator");
              digitalWrite(29,HIGH);
            default:
                currentPacket.ack = 0;
                currentAltPacket.ack = 0;
                break;
            }
        } else {
            currentPacket.ack = 0;
            currentAltPacket.ack = 0;
        }

    } else {
        Serial.println("No reply received");
    }
}

bool Radio::parseReply(const char* reply, int& command, float& argument) {
    // First check if the packet has GS: prefix
    if (strncmp(reply, "GS:", 3) == 0) {
        // Skip the "GS:" prefix
        const char* dataStart = reply + 3;
        
        char* endPtr;
        // Parse command (integer before comma)
        command = strtol(dataStart, &endPtr, 10);
        if (*endPtr != ',') {
            return false;
        }
        
        // Skip the comma
        endPtr++;
        
        // Parse argument (float after comma)
        argument = strtof(endPtr, &endPtr);
        
        // Allow for :END suffix or null termination
        if (strncmp(endPtr, ":END", 4) == 0 || *endPtr == '\0' || *endPtr == '\r' || *endPtr == '\n') {
            return true;
        }
    }
    
    return false;
}


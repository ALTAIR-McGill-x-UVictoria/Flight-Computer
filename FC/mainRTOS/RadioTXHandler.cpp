#include "RadioTXHandler.h"
#include "utils.h"  // Include to access buzzer and calibration functions
#include <Arduino.h>
#include <stdlib.h>

#define BUZZER_PIN 22

extern radioPacket currentPacket;  // Access the global struct

Radio::Radio() : timeout(500) {
    rf95 = new RH_RF95(RFM95_CS, RFM95_INT);
}

Radio::~Radio() {
    delete rf95;
}

void Radio::setup() {
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    // Manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if (!rf95->init()) {
        // Serial.println("LoRa radio init failed");
        while (1);
    }

    if (!rf95->setFrequency(RF95_FREQ)) {
        // Serial.println("setFrequency failed");
        while (1);
    }

    rf95->setSignalBandwidth(BW);
    rf95->setSpreadingFactor(SF);
    rf95->setTxPower(TX_POWER, false);
}

void Radio::radioTx(const char* packet) {
    rf95->send((uint8_t*)packet, strlen(packet));
    rf95->waitPacketSent();
}

bool Radio::radioRx(char* buffer, uint8_t* len) {
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
    // Serial.println("Packet sent");

    // Check for reply with shorter timeout
    if (radioRx(reply, &len)) {
        // Serial.print("Reply received: ");
        // Serial.println(reply);
        lastRSSI = rf95->lastRssi();
        lastSNR = rf95->lastSNR();

        int command;
        float argument;
        
        if (parseReply(reply, command, argument)) {
            currentPacket.ack = command;
            
            switch (command) {
            case 1:
                playBuzzer("warning");  // Play buzzer with warning pattern
                break;
            // other cases...
            case 6:
                calibrateIMU();
                break;
            default:
                break;
            }
        } else {
            currentPacket.ack = 0;
        }
    } else {
        // Serial.println("No reply received");
    }
}

bool Radio::parseReply(const char* reply, int& command, float& argument) {
    char* endPtr;
    
    // Parse command (integer before comma)
    command = strtol(reply, &endPtr, 10);
    if (*endPtr != ',') {
        return false;
    }
    
    // Skip the comma
    endPtr++;
    
    // Parse argument (float after comma)
    argument = strtof(endPtr, &endPtr);
    if (*endPtr != '\0' && *endPtr != '\r' && *endPtr != '\n') {
        return false;
    }
    
    return true;

}


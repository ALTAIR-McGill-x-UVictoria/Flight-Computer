//radio logic code

#ifndef radioLogic.h 

#define radioLogic.h 


#include "Arduino.h"

#include <SPI.h>
#include <RH_RF95.h>
#include <Arduino.h>

#define RFM95_RST 5
#define RFM95_CS 10
#define RFM95_INT 4


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ   433.0
#define RF95_POWER  13

class radioLogic {

    public:

    RH_RF95 rf95(int 10, int 4);

    void initializeRadio(){
      Serial.println("Initializing SX1276");

      pinMode(RFM95_RST, OUTPUT);
      digitalWrite(RFM95_RST, HIGH);
      
      while (!Serial) delay(1);
      delay(100);

      Serial.println("Feather LoRa TX Test!");
      digitalWrite(RFM95_RST, LOW);
      delay(10);
      digitalWrite(RFM95_RST, HIGH);
      delay(10);

      while (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
        while (1);
      }
      Serial.println("LoRa radio init OK!");


      // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
      if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
      }

      Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

      rf95.setTxPower(RF95_POWER, false);
    }

    
    radioLogic(){
      initializeRadio();

    }
    
      void radioLogic::txLoop()(){

      delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
      Serial.println("Transmitting..."); // Send a message to rf95_server

      char radiopacket[20] = "Hello World #      ";
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending "); Serial.println(radiopacket);
      radiopacket[19] = 0;

      Serial.println("Sending...");
      delay(10);
      rf95.send((uint8_t *)radiopacket, 20);

      Serial.println("Waiting for packet to complete...");
      delay(10);
      rf95.waitPacketSent();
      // Now wait for a reply
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      Serial.println("Waiting for reply...");
      if (rf95.waitAvailableTimeout(1000)) {
        // Should be a reply message for us now
        if (rf95.recv(buf, &len)) {
          Serial.print("Got reply: ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);
        } else {
          Serial.println("Receive failed");
        }
      } else {
        Serial.println("No reply, is there a listener around?");
      }


    }

    public void radioLogic::rxLoop()(){

    if (rf95.available()) {
        // Should be a message for us now
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
          digitalWrite(LED_BUILTIN, HIGH);
          RH_RF95::printBuffer("Received: ", buf, len);
          Serial.print("Got: ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);

          // Send a reply
          uint8_t data[] = "And hello back to you";
          rf95.send(data, sizeof(data));
          rf95.waitPacketSent();
          Serial.println("Sent a reply");
          digitalWrite(LED_BUILTIN, LOW);
        } else {
          Serial.println("Receive failed");
        }
      }

    }

};

#endif
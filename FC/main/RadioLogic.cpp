#include "RadioLogic.h"
#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>


void RadioLogic::initializeRadio(){

  Serial.println("Initializing SX1276");

    // RH_RF95 rf95(RFM95_CS, RFM95_INT);
    // RH_RF95 rf95 = RadioLogic.rf95;
    // RH_RF95 rad = this.rf95;

    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
      
    while (!Serial) delay(1);
    delay(100);

    Serial.println("Feather LoRa TX Test!");
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    // Serial.println("DEBUG");

    while (!this->rf95.init()) {
      Serial.println("LoRa radio init failed");
      Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
      while (1);
    }
    Serial.println("LoRa radio init OK!");


    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!this->rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
      while (1);
    }

    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

    this->rf95.setTxPower(RF95_POWER, false);
}

void RadioLogic::radioTx(){


  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server

  char radiopacket[20] = "Hello World";
  // itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;

  Serial.println("Sending...");
  delay(10);
  this->rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete...");
  delay(10);

  // radio.rf95.mode();
  this->rf95.waitPacketSent();
  Serial.println("==DEBUG==");
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (this->rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (this->rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(this->rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }

}

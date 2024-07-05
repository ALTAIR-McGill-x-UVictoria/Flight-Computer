//Flight Computer Main

// #include <RadioLib.h>
#include <ArduinoQueue.h>
#include "Waveshare_10Dof-D.h"
#include "RadioLogic.h"

RadioLogic radio;

RH_RF95 rf95(RFM95_CS, RFM95_INT);
// RH_RF95 rf95;

void setup() {

  // radio = new RadioLogic();
  // rf95 = radio.rf95
  
  Serial.begin(115200);
  Serial.println("Initializing");
  radio.initializeRadio();
  

  Serial.println("Running main loop");


}

void loop() {
  Serial.println(rf95.getDeviceVersion());//debugging tool
  // put your main code here, to run repeatedly:
  
  // radio.radioTx();


//test
  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server

  // char radiopacket[20] = "Hello World";
  char radiopacket[20] = "Hello World #      ";
  // itoa(packetnum++, radiopacket+13, 10);
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

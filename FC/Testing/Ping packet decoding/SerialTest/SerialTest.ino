// #include "uart.h"
#include <SD.h>

#define PingSerial Serial3
#define PingBaud 56700

unsigned int Crc16Table[256];

void setup() {
  // put your setup code here, to run once:
  // Serial1.begin(57600);
  while (!Serial && (millis() < 3000));

  PingSerial.begin(PingBaud);
  // while (!PingSerial && (millis() < 3000));

  // crcinit();

  Serial.println("Setup complete");
  // uart_init(56700);
  // SDSetup();

}

void loop() {
  
  // Serial.println("test");
  unsigned int incomingByte = 0;

  String str = "";

    if (PingSerial.available() > 0) {
      incomingByte = PingSerial.read();
      // Serial.print("USB received: ");
      String hex = String(incomingByte, HEX);
      // int i = crcCompute(hex.c_str(), 1);
      // hex.toUpperCase();
      // int ascii = hex_to_ascii(hex[0], hex[1]);
      // String hexout = String(i, HEX);
      Serial.print(hex + "-");
      // Serial.print(String(i) + "-");
      // HWSERIAL.print("USB received:");
      // HWSERIAL.println(incomingByte, DEC);
    }
  
  // SDWrite(incomingByte);
}



void crcinit(){
  unsigned int i, bitctr, crc;
  for (i = 0; i < 256; i++){
    crc = (i << 8);
    for (bitctr = 0; bitctr < 8; bitctr++){
      crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
    }
    Crc16Table[i];
  }
}

unsigned int crcCompute(unsigned char *block, unsigned long int length){
  unsigned long int i;
  unsigned int crc = 0;
  for (i = 0; i < length; i++){
    crc = Crc16Table[crc >> 8] ^ (crc << 8) ^ block[i];
  }
  return crc;
}

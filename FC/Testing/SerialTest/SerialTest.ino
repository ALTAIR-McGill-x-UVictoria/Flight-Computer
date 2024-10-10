// #include "uart.h"
#include <SD.h>

#define PingSerial Serial2

void setup() {
  // put your setup code here, to run once:
  // Serial1.begin(57600);

  
  // uart_init(56700);
  SDSetup();

}

void loop() {
  
  // Serial.println("test");
  int incomingByte = 0;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    // Serial.print("USB received: ");
    // Serial.println(incomingByte, DEC);
    // HWSERIAL.print("USB received:");
    // HWSERIAL.println(incomingByte, DEC);
  }
  SDWrite(incomingByte);
}

void SDSetup() {
  // Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    // Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }

  // Serial.println("Card initialized.");
  // int i = 0;
  // while(SD.exists(("datalog_" + String(i) + ".txt").c_str())){
  //   i++;
  // }
  // currentFilePath = "datalog_" + String(i) + ".txt";
}

void SDWrite(String log) {
  File dataFile = SD.open("test.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(": ");
    dataFile.println(log);
    dataFile.close();
// print to the serial port too:
    // Serial.println(log);

  }

}

//Flight Computer Main

// #include <RadioLib.h>
#include <ArduinoQueue.h>
#include "Waveshare_10Dof-D.h"
#include "RadioLogic.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"

bool gbSenserConnectState = false;

#define RFM95_RST 5
#define RFM95_CS 10
#define RFM95_INT 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

void setup() {

  // radio = new RadioLogic();
  // rf95 = radio.rf95
  
  Serial.begin(115200);
  Serial.println("Initializing");
  radioSetup();
  sensorSetup();
  

  Serial.println("Running main loop");


}

void loop() {
  // Serial.println(radio.rf95.getDeviceVersion());//debugging tool
  // put your main code here, to run repeatedly:
  fullSensorLoop();
  radioTxLoop();
  

}




void radioSetup(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
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

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(20, false);

}

void radioTxLoop(){

  // delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server

  // char radiopacket[20] = "Hello World";
  char radiopacket[20] = "Hello World";
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

void sensorSetup(){
  bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  Serial.begin(115200);

  imuInit(&enMotionSensorType, &enPressureType);
  if(IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
  {
    Serial.println("Motion sersor is ICM-20948");
  }
  else
  {
    Serial.println("Motion sersor NULL");
  }
  if(IMU_EN_SENSOR_TYPE_BMP280 == enPressureType)
  {
    Serial.println("Pressure sersor is BMP280");
  }
  else
  {
    Serial.println("Pressure sersor NULL");
  }
  delay(1000);

}

void fullSensorLoop(){

  imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  Serial.println();
  Serial.println("/-------------------------------------------------------------/");
  Serial.print("Roll : "); Serial.print(stAngles.fRoll);
  Serial.print("    Pitch : "); Serial.print(stAngles.fPitch);
  Serial.print("    Yaw : "); Serial.print(stAngles.fYaw);
  Serial.println();
  Serial.print("Acceleration: X : "); Serial.print(stAccelRawData.s16X);
  Serial.print("    Acceleration: Y : "); Serial.print(stAccelRawData.s16Y);
  Serial.print("    Acceleration: Z : "); Serial.print(stAccelRawData.s16Z);
  Serial.println();
  Serial.print("Gyroscope: X : "); Serial.print(stGyroRawData.s16X);
  Serial.print("       Gyroscope: Y : "); Serial.print(stGyroRawData.s16Y);
  Serial.print("       Gyroscope: Z : "); Serial.print(stGyroRawData.s16Z);
  Serial.println();
  Serial.print("Magnetic: X : "); Serial.print(stMagnRawData.s16X);
  Serial.print("      Magnetic: Y : "); Serial.print(stMagnRawData.s16Y);
  Serial.print("      Magnetic: Z : "); Serial.print(stMagnRawData.s16Z);
  Serial.println();
  Serial.print("Pressure : "); Serial.print((float)s32PressureVal / 100);
  Serial.print("     Altitude : "); Serial.print((float)s32AltitudeVal / 100);
  Serial.println();  
  Serial.print("Temperature : "); Serial.print((float)s32TemperatureVal / 100);
  Serial.println();  
  delay(100);
}

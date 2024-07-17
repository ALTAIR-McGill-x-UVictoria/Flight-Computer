
//Flight Computer Main

// #include <RadioLib.h>
#include <ArduinoQueue.h>
#include "Waveshare_10Dof-D.h"
// #include "RadioLogic.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <SD.h>

bool gbSenserConnectState = false;

#define RFM95_RST 5
#define RFM95_CS 10
#define RFM95_INT 4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0


//LED pin definitions
#define PWM_LED 3

//Battery voltage definitions
#define ANALOG_IN_PIN 23
float R2 = 3485;
float R1 = 10040;
float voltage;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

elapsedMillis sendTimer;

volatile bool enableSDWrite = 0;

//Command parser variables
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use by strtok() function

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
int cmdCode = 0;
float floatArg = 0.0;

String commandPacket;

boolean newData = false;

//DEBUGGING
#define RX_ENABLE 1
#define LED_ENABLE 0
#define DAQ_DEBUG 0

#define LOOP_TIMER 1000

volatile int pong_flag = 0;


volatile int ledStatus = 0;
volatile int ledIntensity = 0;

char * packetToSend;


void setup() {

  // radio = new RadioLogic();
  // rf95 = radio.rf95
  
  // Serial.begin(115200);
  while (!Serial && (millis() < 3000));

  Serial.println("Initializing");
  radioSetup();
  sensorSetup();
  SDSetup();


  pinMode(ANALOG_IN_PIN, INPUT); //voltage sensor
  pinMode(PWM_LED, OUTPUT);

  

  Serial.println("Running main loop");
  SDWrite("System init complete");

}

void loop() {
  // Serial.println(radio.rf95.getDeviceVersion());//debugging tool
  // put your main code here, to run repeatedly:

  if(DAQ_DEBUG){
    fullSensorLoop();
  }
  
  if(enableSDWrite == 1){
    
    SDWrite(formRadioPacket(1));
    // Serial.println("Write");
    
  }

  if(RX_ENABLE){
    
    if(sendTimer >= 1000){
      radioTx(formRadioPacket(1));
      sendTimer = 0;
    }
  }



  if(LED_ENABLE == 1){
    LEDhandler();
  }


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

void radioTx(char radiopacket[100]){

  // delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  // Serial.println("Transmitting..."); // Send a message to rf95_server

  // char radiopacket[20] = "Hello World";
  // itoa(packetnum++, radiopacket+13, 10);
  // Serial.print("Sending "); Serial.println(radiopacket);
  // radiopacket[19] = 0;

  // Serial.println("Sending...");
  // delay(10);
  rf95.send((uint8_t *)radiopacket, 100);

  // Serial.println("Waiting for packet to complete...");
  // delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(200)) { //test if necessary, blocks all other processes (annoying)
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      // Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      FCpacketParser((char*)buf); //parse and execute the received packet

      //debugging
      // Serial.print("LED: "); Serial.println(ledStatus);

    } else {
      // Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }
}

void sensorSetup(){
  bool bRet;
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  // Serial.begin(115200);

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
  delay(200);

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
  // delay(100);
}

void parseData() {

      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    // Serial.print(strtokIndx); Serial.print("-"); Serial.println(strtokIndx != NULL);
    if(NULL != strtokIndx)
    {
    cmdCode = atoi(strtokIndx);   // convert this part to an integer
    }

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    
    if(NULL != strtokIndx)
    {
      floatArg = atof(strtokIndx);     // convert this part to an integer
    }

    
}

void SDSetup(){
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("Card initialized.");
}

void SDWrite(String log){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(millis()); dataFile.print(": "); dataFile.println(log);
    dataFile.close();
    // print to the serial port too:
    Serial.println(log);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}

char* formRadioPacket(bool enable_daq){ //includes DAQ
  if(enable_daq == 1){

    String packet = "";
    imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
    pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

    packet = packet + battery_voltage() + "," + stAngles.fPitch + "," + stAngles.fRoll + "," + stAngles.fYaw + "," + 
    (float)s32PressureVal / 100 + "," + (float)s32AltitudeVal / 100 + "," + (float)s32TemperatureVal / 100 + "," +
    ledStatus + "," + ledIntensity + "," + enableSDWrite + "," + pong_flag;

    pong_flag = 0;

    return packet.c_str();
  }
}

void FCpacketParser(char* packet){
    int commandCode;
    float commandArg;
        // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index
    // char * temp;
    // strcpy(temp,packet);


    strtokIndx = strtok(packet,",");      // get the first part - the string
    // Serial.print(strtokIndx); Serial.print("-"); Serial.println(strtokIndx != NULL);
    if(NULL != strtokIndx)
    {
      commandCode = atoi(strtokIndx);   // convert this part to an integer
    }

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    
    if(NULL != strtokIndx)
    {
      commandArg = atof(strtokIndx);     // convert this part to an integer
    }
    
    switch (commandCode) {  
    
      case 1:
        //ping
        // Serial.println("              pong");
        pong_flag = 1;
      break;

      case 2:
        //ledon
        ledStatus = 1;
        ledIntensity = (int)(commandArg / 100.0f * 255.0f);
        // Serial.print("LED ON at "); Serial.println(ledIntensity);

      break;

      case 3:
        //ledoff
        ledStatus = 0;
        // Serial.println("LED OFF");
      
      break;

      case 4: 
        //dangle
        //TODO
      break;

      case 5:
        //sdwrite
        enableSDWrite = 1;
      break;

      case 6:
        //sdstop
        enableSDWrite = 0;
      break;

      default:
      break;

    }

}

float battery_voltage() {
  int adcValue = analogRead(ANALOG_IN_PIN);
  float resistorRatio = R2/(R2+R1);
  float conversionFactor = 3.3/(1024*resistorRatio);
  float voltage = (float)(adcValue*conversionFactor);
  return voltage;
}

void LEDhandler(){
  if(ledStatus == 1){
    analogWrite(PWM_LED, ledIntensity);
  } else{
    analogWrite(PWM_LED, 0);
  }

}
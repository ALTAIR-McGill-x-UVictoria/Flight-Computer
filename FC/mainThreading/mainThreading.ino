//Flight Computer Main

#include <ArduinoQueue.h>
#include "Waveshare_10Dof-D.h"
// #include "RadioLogic.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <SD.h>
#include <Bonezegei_DRV8825.h>
#include <TeensyThreads.h>

//Callsign
#define CALLSIGN "VA2ETD"

//Radio pin definitions
#define RFM95_RST 20  //5
#define RFM95_CS 37
#define RFM95_INT 21  //4

//LoRa parameters definitions
#define RF95_FREQ 433.0
#define SF 8
#define BW 125000
#define TX_POWER 20

//LED pin definitions
#define PWM_LED1 14
#define PWM_LED2 2
#define PWM_LED3 15

//Battery voltage definitions
#define MAIN_BATTERY_PIN 23
#define MOTOR_BATTERY_PIN 30
#define HEATING_BATTERY_PIN 31
float R2 = 3520;
float R1 = 10000;
float voltage;

//Functionality enable definitions
#define RX_ENABLE 1
#define DAQ_ENABLE 1
#define SD_ENABLE 0
#define LED_ENABLE 0
#define STEPPER_ENABLE 0
#define ACTUATOR_ENABLE 0
#define HEATING_ENABLE 0
#define DAQ_DEBUG 0
#define SD_DEBUG 0
#define PING_ENABLE 0

// in psixx
#define PRESSURE_TO_RELEASE 264

//Radio loop timer (fastest transmission speed)
#define LOOP_TIMER 200
#define FLIGHT_MODE_TIMER 200
#define FLIGHT_MODE_COUNT 3
#define SD_TIMER 50

#define ENABLE_SERIAL 0  //Enable Serial port

//Stepper motor definitions
//TO CHANGE
#define DIR_PIN 39
#define STEP_PIN 38
// // #define SLEEP_PIN 4 //set to 3.3
// // #define RESET_PIN 5 //set to 3.3
#define FAULT_PIN 40

#define M0_PIN 33
#define M1_PIN 34
#define M2_PIN 35


//#define DIR_PIN 2
//#define STEP_PIN 3
//#define SLEEP_PIN 4
//#define RESET_PIN 5
//#define FAULT_PIN 6

//#define M0_PIN 7
//#define M1_PIN 8
//#define M2_PIN 9

#define STEPS_PER_REV 200

#define NUM_LEDS 3

#define CW 0
#define CCW 1

#define USER 0
#define SYS 1

// Ping200XR
#define CRC_POLYNOMIAL 0x1021
#define FLAG_BYTE 0x7E
#define ESCAPE_BYTE 0x7D

uint16_t Crc16Table[256];

#define PingSerial Serial7
#define PingBaud 56700

const int maxPacketSize = 64;      // Define the maximum size for each packet (adjust as needed)
byte packetBuffer[maxPacketSize];  // Buffer to store the complete packet
byte foundPacket[maxPacketSize];   // Separate buffer to store the detected packet
int packetIndex = 0;               // Index to track position in packetBuffer
bool capturing = false;            // Flag to check if we are capturing a packet
int foundpacketindex = 0;

// struct definitions



struct HeartbeatMessage {
  uint8_t gnss_valid;
  uint8_t maintenance_req;
  uint8_t ident_active;
  uint8_t initialized;
};

struct OwnshipReport {
  uint8_t messageID;
  uint8_t trafficAlertStatus;
  uint8_t addressType;
  char participantAddress[7];
  float latitude;
  float longitude;
  int altitude;
  uint8_t miscellaneousIndicators;
  uint8_t NIC;
  uint8_t NACp;
  uint16_t horizontalVelocity;
  uint16_t verticalVelocity;
  uint8_t trackHeading;
  uint8_t emitterCategory;
  char flightIdentification[7];
};

struct GeometricAltitude {
  uint8_t messageID;
  int16_t geometricAltitude;
};

struct GNSSData {
  uint8_t messageID;
  uint8_t messageVersion;
  uint32_t utcTime;
  double latitude;
  double longitude;
  double altitude;
  double hpl;
  double vpl;
  double hfom;
  double vfom;
  double hvfom;
  double vvfom;
  double gnssVerticalSpeed;
  double northSouthVelocity;
  double eastWestVelocity;
};

struct TransponderStatus {
  uint8_t messageID;
  uint8_t messageVersion;
  bool txEnabled;
  bool identButtonActive;
};

struct BarometerSensor {
  uint8_t messageID;
  uint8_t sensorType;
  float barometricPressure;
  int32_t barometricPressureAltitude;
  float barometricSensorTemperature;
};

//end of definitions


HeartbeatMessage heartbeatMessage;
OwnshipReport ownshipReport;
GeometricAltitude geometricAltitude;
GNSSData gnssData;
TransponderStatus transponderStatus;
BarometerSensor barometerSensor;


// Singleton instances
RH_RF95 rf95(RFM95_CS, RFM95_INT);

IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

Bonezegei_DRV8825 stepper(DIR_PIN, STEP_PIN);


volatile bool enableSDWrite = 0;
String currentFilePath = "datalog_0.txt";

//Command parser variables
const byte numChars = 32;
// char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use by strtok() function

// variables to hold the parsed data
int cmdCode = 0;
float floatArg = 0.0;

//Local variables
String commandPacket;

volatile int reception_confirm = 0;
String commandid = 0;
String recvdCommandid = 0;

volatile int led1Status = 0;
volatile int led2Status = 0;
volatile int led3Status = 0;
volatile int ledIntensity = 0;
int toggleLEDblink = 0;

elapsedMillis sendTimer;
elapsedMillis LEDtimer;
elapsedMillis timeSinceLastGroundPacket;
elapsedMillis sdTimer;

float LEDtimerInput = 0;

char* packetToSend;
char* packetForm;

int toggleLongPacket = 1;
int toggleFlightMode = 0;
int transmissionCounter = 0;
int timeout = 3000;
int flightModeCount = FLIGHT_MODE_COUNT;

//Stepper motor variables
float angleToSet = 0;
int speed = 2000;
int step_division = 4;

double partial_steps = 0;
bool curr_dir = CW;
int steps_left = 0;
bool step_lock = false;
int total_steps = 0;

float payload_yaw = 0;
float last_payload_yaw = 0;
bool toggle_yaw_stabilization = false;

double currentStepperAngle = 0;

bool actuatorStatus = 0;
#define ACTUATOR_PIN 5
bool heatingStatus = 0;
#define HEATING_PIN 39

#define LED_PIN 13
bool led_state = false;

int stepperThread = 0;
int sdThread = 99;

void crc_init();
void PingHandler();
int set_abs_angle(double angle);
void reboot();

void setup() {

#if ENABLE_SERIAL
  // Serial.begin(115200);
  while (!Serial && (millis() < 3000))
    ;
  Serial.println("Initializing");
#endif

#if RX_ENABLE
  radioSetup();
#endif

#if DAQ_ENABLE
  sensorSetup();
  // threads.addThread(DAQacquire);
#endif

#if SD_ENABLE
  SDSetup();
  // SDNewFile();
#endif

#if LED_ENABLE
  pinMode(PWM_LED1, OUTPUT);
  pinMode(PWM_LED2, OUTPUT);
  pinMode(PWM_LED3, OUTPUT);
  threads.addThread(LEDhandler);
#endif

  pinMode(7, OUTPUT);


  // pinMode(ANALOG_IN_PIN, INPUT); //voltage sensor

#if SD_ENABLE
  SDWrite("System init complete");
#endif

#if ENABLE_SERIAL
  Serial.println("Init complete");
#endif

#if STEPPER_ENABLE
  stepperSetup();
  stepperThread = threads.addThread(stepperHandler);
#endif

#if ACTUATOR_ENABLE
  pinMode(ACTUATOR_PIN, OUTPUT);
#endif

#if HEATING_ENABLE
  pinMode(HEATING_PIN, OUTPUT);
#endif

#if PING_ENABLE
  PingSerial.begin(PingBaud);
  crc_init();
  int pingThread = threads.addThread(PingHandler);
  // Serial.println(pingThread);
#endif

  int loopThread = threads.addThread(loopHandler);
}

void loop() {
}


void DAQacquire() {
  while (1) {
    imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
    pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
  }
}

/*
=========================
Function definitions
=========================
*/

void SDHandler() {
  while (1) {
    if (sdTimer >= SD_TIMER) {
      SDWrite(logger());
      sdTimer = 0;
    }
  }
}

void loopHandler() {

  while (1) {

    imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
    pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
#if SD_ENABLE
    if (enableSDWrite == 1) {
      SDWrite(logger());
    }
#endif

#if RX_ENABLE
    commandid = String(rand()).substring(0, 4);

    if (toggleFlightMode == 0) {

      if (sendTimer >= LOOP_TIMER) {

        radioTx(formRadioPacket(toggleLongPacket, commandid));

        sendTimer = 0;
      }

    } else {

      if (sendTimer >= FLIGHT_MODE_TIMER) {

        radioTx(formRadioPacket(toggleLongPacket, commandid));

        sendTimer = 0;
      }
    }


#endif

    // #if LED_ENABLE
    //   LEDhandler();

    //   //dont forget to remove

    // #endif

#if ACTUATOR_ENABLE
    actuatorHandler();
#endif

#if HEATING_ENABLE
    heatingHandler();
#endif

#if STEPPER_ENABLE
    if (battery_voltage("main") < 11.3) { threads.kill(stepperThread); }
#endif
  }
}



void radioSetup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1)
      ;
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }

#if ENABLE_SERIAL
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
#endif

  rf95.setSignalBandwidth(BW);
  rf95.setSpreadingFactor(SF);
  rf95.setTxPower(TX_POWER, false);
  // rf95.setModemConfig(rf95.)
}

void radioTx(char radiopacket[150]) {

  rf95.send((uint8_t*)radiopacket, toggleLongPacket ? 150 : 20);

  rf95.waitPacketSent();

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (toggleFlightMode == 1 && transmissionCounter < flightModeCount) {
    transmissionCounter++;
    return;
  } else {
    transmissionCounter = 0;
    if (rf95.waitAvailableTimeout(timeout)) {  //test if necessary, blocks all other processes (annoying)
      // Should be a reply message for us now
      if (rf95.recv(buf, &len)) {
// Serial.print("Got reply: ");
#if ENABLE_SERIAL
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.print(rf95.lastRssi(), DEC);
        Serial.print(", SNR: ");
        Serial.println(rf95.lastSNR(), DEC);
#endif

        FCpacketParser((char*)buf);  //parse and execute the received packet
        timeSinceLastGroundPacket = 0;

      } else {
        // Serial.println("Receive failed");
      }
    } else {
#if ENABLE_SERIAL
      Serial.println("No reply, is there a listener around?");
#endif
    }
  }
}

void sensorSetup() {
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  // Serial.begin(115200);

  imuInit(&enMotionSensorType, &enPressureType);
  if (IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType) {
    // Serial.println("Motion sersor is ICM-20948");
  } else {
    // Serial.println("Motion sersor NULL");
  }
  if (IMU_EN_SENSOR_TYPE_BMP280 == enPressureType) {
    // Serial.println("Pressure sersor is BMP280");
  } else {
    // Serial.println("Pressure sersor NULL");
  }
  delay(200);
}


void parseData() {

  // split the data into its parts
  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ":");
  strtokIndx = strtok(NULL, ",");
  // Serial.print(strtokIndx); Serial.print("-"); Serial.println(strtokIndx != NULL);
  if (NULL != strtokIndx) {
    cmdCode = atoi(strtokIndx);  // convert this part to an integer
  }

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off

  if (NULL != strtokIndx) {
    floatArg = atof(strtokIndx);  // convert this part to an integer
  }
}

void SDSetup() {
  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    return 1; //fail
  }

  // Serial.println("Card initialized.");
  int i = 0;
  while (SD.exists(("datalog_" + String(i) + ".txt").c_str())) {
    i++;
  }
  currentFilePath = "datalog_" + String(i) + ".txt";

  return 1;
}

void SDWrite(String log) {
  File dataFile = SD.open(currentFilePath.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(": ");
    dataFile.println(log);
    dataFile.close();
// print to the serial port too:
#if SD_DEBUG
#if ENABLE_SERIAL
    Serial.println(log);
#endif
#endif

  } else {
    // if the file isn't open, pop up an error:
    // Serial.println("error opening datalog.txt");
  }
}

void SDNewFile() {
  int i = 0;
  String loghead = "datalog_";
  while (SD.exists((loghead + String(i) + ".txt").c_str())) {
    i++;
  }
  currentFilePath = loghead + String(i++) + ".txt";
}

String logger() {
  String log = "";
  log = log + ":";
  // very inefficient, works
  String logarr[] = {
    // Radio
    String(rf95.lastRssi()),
    String(rf95.lastSNR()),
    String(timeSinceLastGroundPacket),
    // Status indicators
    String(battery_voltage("main")),
    String(battery_voltage("motor")),
    String(led1Status),
    String(led2Status),
    String(led3Status),
    String(ledIntensity),
    String(currentStepperAngle),
    String(actuatorStatus),
    // Ping200XR
    String(ownshipReport.latitude, DEC),
    String(ownshipReport.longitude, DEC),
    String(ownshipReport.altitude),
    String(ownshipReport.horizontalVelocity),
    String(ownshipReport.verticalVelocity),
    String(ownshipReport.flightIdentification),
    String(heartbeatMessage.ident_active),
    String(barometerSensor.barometricPressure),
    String(barometerSensor.barometricPressureAltitude),
    String(barometerSensor.barometricSensorTemperature),
    // IMU
    String(stAngles.fRoll),
    String(stAngles.fPitch),
    String(stAngles.fYaw),
    String(stAccelRawData.s16X),
    String(stAccelRawData.s16Y),
    String(stAccelRawData.s16Z),
    String(stGyroRawData.s16X),
    String(stGyroRawData.s16Y),
    String(stGyroRawData.s16Z),
    String(stMagnRawData.s16X),
    String(stMagnRawData.s16Y),
    String(stMagnRawData.s16Z),
    String((float)s32PressureVal / 100),
    String((float)s32AltitudeVal / 100),
    String((float)s32TemperatureVal / 100)

  };
  for (String str : logarr) {
    log = log + str + ",";
  }

  // reception_confirm + "," + rf95.lastRssi() + "," + rf95.lastSNR() + "," + battery_voltage("main") + "," + stAngles.fPitch + ","
  // + stAngles.fRoll + "," + stAngles.fYaw + "," + stAccelRawData.s16X + "," + stAccelRawData.s16Y + "," + stAccelRawData.s16Z + "," + (float)s32PressureVal / 100 + ","
  // + (float)s32AltitudeVal / 100 + "," + (float)s32TemperatureVal / 100 + "," + led1Status + led2Status + led3Status + "," + ledIntensity + "," + enableSDWrite + "," + heatingStatus + "," + actuatorStatus + ","
  // + battery_voltage("motor") + "," + String(ownshipReport.flightIdentification) + "," + String(ownshipReport.latitude,DEC) + "," + String(ownshipReport.longitude,DEC) + "," + String(ownshipReport.altitude)
  return log;
}

char* formRadioPacket(bool enable_long, String cmdid) {

  //FOR FUTURE, FORM AS STRUCT AND TRANSMIT WITHOUT COMMAS

  String packet = "";

  digitalWrite(7, reception_confirm == 1 ? HIGH : LOW);

  if (enable_long == 1) {

    //very inefficient, works fine
    packet = packet + cmdid + ":" + reception_confirm + "," + rf95.lastRssi() + "," + rf95.lastSNR() + "," + battery_voltage("main") + "," + stAngles.fPitch + ","
             + stAngles.fRoll + "," + stAngles.fYaw + "," + stAccelRawData.s16X + "," + stAccelRawData.s16Y + "," + stAccelRawData.s16Z + "," + (float)s32PressureVal / 100 + ","
             + (float)s32AltitudeVal / 100 + "," + (float)s32TemperatureVal / 100 + "," + led1Status + led2Status + led3Status + "," + ledIntensity + "," + enableSDWrite + "," + heatingStatus + "," + actuatorStatus + ","
             + battery_voltage("motor") + "," + String(ownshipReport.flightIdentification) + "," + String(ownshipReport.latitude, DEC) + "," + String(ownshipReport.longitude, DEC) + "," + String(ownshipReport.altitude);

    reception_confirm = 0;
    // Serial.println(ownshipReport.flightIdentification);

    return packet.c_str();
  }

  //default small packet
  packet = packet + cmdid + ":" + reception_confirm + "," + rf95.lastRssi() + "," + rf95.lastSNR() + "," + battery_voltage("main") + "," + String(ownshipReport.latitude, DEC) + "," + String(ownshipReport.longitude, DEC) + "," + String(ownshipReport.altitude) + "," +String(ownshipReport.flightIdentification);

  reception_confirm = 0;


  return packet.c_str();
}

void FCpacketParser(char* packet) {
  int commandCode = 0;
  float commandArg = 0.0f;
  // split the data into its parts
  char* strtokIndx;  // this is used by strtok() as an index
  // char * temp;
  // strcpy(temp,packet);



  strtokIndx = strtok(packet, ":");

  if (NULL != strtokIndx) {
    recvdCommandid = strtokIndx;
  }

  strtokIndx = strtok(NULL, ",");  // get the first part - the string
  // Serial.print(strtokIndx); Serial.print("-"); Serial.println(strtokIndx != NULL);
  if (NULL != strtokIndx) {
    commandCode = atoi(strtokIndx);  // convert this part to an integer
  }

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off

  if (NULL != strtokIndx) {
    commandArg = atof(strtokIndx);  // convert this part to an integer
  }

  // Serial.print("IDs:"); Serial.print(commandid); Serial.print(","); Serial.println(recvdCommandid);

  if (commandid != recvdCommandid) {
    reception_confirm = 0;
    return;
  }

  switch (commandCode) {

    case 1:
      //ping
      // digitalWrite(7,HIGH);
      reception_confirm = 1;

      break;

    case 2:
      led1Status = !led1Status;
      reception_confirm = 1;

      break;

    case 3:
      led2Status = !led2Status;
      reception_confirm = 1;

      break;

    case 4:
      led3Status = !led3Status;
      reception_confirm = 1;

      break;

    case 5:
      //ledoff
      led1Status = 0;
      led2Status = 0;
      led3Status = 0;
      reception_confirm = 1;

      break;

    case 6:
      //dangle
      angleToSet = commandArg;
      set_abs_angle((double)angleToSet);

      reception_confirm = 1;

      break;

    case 7:
      //sdwrite
      enableSDWrite = 1;
      // sdThread = threads.addThread(SDHandler);

      reception_confirm = 1;
      break;

    case 8:
      //sdstop
      enableSDWrite = 0;
      // threads.kill(sdThread);

      reception_confirm = 1;
      break;

    case 9:
      SD.remove("datalog.txt");  //to test

      reception_confirm = 1;
      break;

    case 10:
      toggleLEDblink = !toggleLEDblink;
      LEDtimerInput = commandArg;

      reception_confirm = 1;
      break;

    case 11:
      ledIntensity = (int)(commandArg / 100.0f * 255.0f);

      reception_confirm = 1;
      break;

    case 12:
      toggleLongPacket = !toggleLongPacket;

      reception_confirm = 1;
      break;

    case 13:
      currentStepperAngle = 0;

      reception_confirm = 1;
      break;

    case 14:
      speed = (int)commandArg;

      reception_confirm = 1;
      break;

    case 15:
      toggle_yaw_stabilization = !toggle_yaw_stabilization;

      reception_confirm = 1;
      break;

    case 16:
      toggleFlightMode = !toggleFlightMode;

      reception_confirm = 1;
      break;

    case 17:
      timeout = commandArg;

      reception_confirm = 1;
      break;

    case 18:
      reception_confirm = 1;
      reboot();
      break;

    case 19:
      actuatorStatus = 1;
      digitalWrite(ACTUATOR_PIN, HIGH);
      reception_confirm = 1;
      break;

    case 20:
      actuatorStatus = 0;
      digitalWrite(ACTUATOR_PIN, LOW);
      reception_confirm = 1;
      break;

    case 21:
      SDNewFile();
      reception_confirm = 1;
      break;

    case 22:
      heatingStatus = 0;
      reception_confirm = 1;
      break;


    default:
      reception_confirm = 0;
      break;
  }
}

float battery_voltage(String battery_select) {
  float resistorRatio = R2 / (R2 + R1);
  float conversionFactor = 3.3 / (1024 * resistorRatio);

  float mainVoltage = (float)(analogRead(MAIN_BATTERY_PIN) * conversionFactor);
  float motorVoltage = (float)(analogRead(MOTOR_BATTERY_PIN) * conversionFactor);
  float heatingVoltage = (float)(analogRead(HEATING_BATTERY_PIN) * conversionFactor);

  if (battery_select == "main") {
    return mainVoltage;
  } else if (battery_select == "motor") {
    return motorVoltage;
  } else if (battery_select == "heating") {
    return heatingVoltage;
  } else {
    return 0.0;
  }
}

void LEDhandler() {
  while (1) {
    if (toggleLEDblink == 1) {
      //blink


      if (LEDtimer <= LEDtimerInput) {
        //off
        analogWrite(PWM_LED1, 0);
        analogWrite(PWM_LED2, 0);
        analogWrite(PWM_LED3, 0);

      } else if (LEDtimer <= LEDtimerInput * 2) {
        analogWrite(PWM_LED1, ledIntensity * led1Status);
        analogWrite(PWM_LED2, ledIntensity * led2Status);
        analogWrite(PWM_LED3, ledIntensity * led3Status);

      } else if (LEDtimer > 2 * LEDtimerInput) {
        LEDtimer = 0;
      }

    } else {
      //default handle
      analogWrite(PWM_LED1, ledIntensity * led1Status);
      analogWrite(PWM_LED2, ledIntensity * led2Status);
      analogWrite(PWM_LED3, ledIntensity * led3Status);
    }
  }
}

void actuatorHandler() {
  if (((float)s32PressureVal / 100 < float(PRESSURE_TO_RELEASE) && ((float)s32PressureVal / 100 > 50)) || millis() >= 1.5 * 3600000) {
    // 
    // || millis() >= 1.5 * 3600000
    // digitalWrite(ACTUATOR_PIN, a == 1 ? HIGH : LOW);
    digitalWrite(ACTUATOR_PIN, HIGH);
    actuatorStatus = 1;
    // return;
  }
}

void stepperSetup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  //pinMode(RESET_PIN, OUTPUT);
  //pinMode(SLEEP_PIN, OUTPUT);
  pinMode(FAULT_PIN, INPUT);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  // pinMode(LED_PIN, OUTPUT);


  //Disables sleep and reset
  //digitalWrite(RESET_PIN, HIGH);
  //digitalWrite(SLEEP_PIN, HIGH);

  stepper.begin();
  stepper.setSpeed(speed);

  //Motor starts low
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  if (set_substep(step_division)) {
    // Serial.println("Invalid Step Division");
    // Serial.flush();
    // exit(-4);
    // while(1);
  }

  stepper.step(0, 200);
}

void stepperHandler() {
  // if (!digitalRead(FAULT_PIN)) {
  //   // Serial.println("Fault pin low: error. Exiting...");
  //   // Serial.flush();
  //   exit(4);
  // }


  while (1) {
    if (!steps_left) { step_lock = false; }
    //Else, turn
    else {
      // Serial.println(steps_left);
      stepper.step(curr_dir, steps_left);
      total_steps += steps_left;
      steps_left = 0;
    }


    if (toggle_yaw_stabilization && !step_lock) {
      stabilize_yaw();
    }
  }
}

void heatingHandler() {
  heatingStatus = (float)s32TemperatureVal / 100 < 15.0 ? 1 : 0;
  digitalWrite(HEATING_PIN, heatingStatus);
}

int set_dir(bool dir) {
  digitalWrite(DIR_PIN, dir);
  curr_dir = dir;
  return 0;
}

int turn_steps(double steps) {
  //Serial.print("Steps: "); Serial.println(steps);
  if (steps == 0) { return 0; }

  //Allows the user to rotate even when stabilizing (Changing the target angle)    //TODO What to do for partial steps?
  if (toggle_yaw_stabilization) {

    //Motor Direction and User rotation in same direction
    if ((steps < 0 && curr_dir == CCW) || (steps > 0 && curr_dir == CW)) {
      steps_left += abs(steps);
    }
    //In different direction
    else {
      steps_left = abs(steps) - steps_left;
      set_dir(!curr_dir);
    }

    return 0;
  }

  //Ignore command if motor already turning
  if (step_lock) { return -1; }

  //Select direction based on sign
  bool dir = (steps > 0) ? CW : CCW;
  set_dir(dir);
  // Serial.print("Steps is: "); Serial.print(steps); Serial.print(" so dir is: "); Serial.println(dir);

  //Update the accumulated error.
  partial_steps += steps - (int)steps;

  //The accumulated error reached 1 or -1. Add it to the steps to do
  if (abs(partial_steps) >= 1) {
    steps += (int)partial_steps;
    partial_steps -= (int)partial_steps;
  }
  //Select direction based on sign (again)
  dir = (steps > 0) ? CW : CCW;
  set_dir(dir);

  steps_left = abs((int)steps);
  step_lock = true;
  return 0;
}

//Turn by X.x degrees. Positive is CW, Negative is CCW
int turn_degrees(double degrees) {
  double steps = (degrees / 360) * STEPS_PER_REV * step_division;
  return turn_steps(steps);
}

int set_abs_angle(double angle) {
  double diff = (angle - currentStepperAngle);  //consider taking modulo
  double steps = (diff / 360) * STEPS_PER_REV * (double)step_division;
  currentStepperAngle = angle;
  //Serial.println(steps);
  return turn_steps(steps);
}

//Go to previous or next LED
int turn_led(bool dir) {
  double steps = (STEPS_PER_REV * step_division / NUM_LEDS);
  if (dir == CCW) steps *= -1;

  return turn_steps(steps);
}

int stabilize_yaw() {
  double delta_angle = last_payload_yaw - payload_yaw;
  last_payload_yaw = payload_yaw;
  // Serial.print("Stabilizing: ");
  // Serial.println(delta_angle);
  turn_degrees(delta_angle);

  return 0;
}

int set_substep(int division) {

  bool M0 = 0;
  bool M1 = 0;
  bool M2 = 0;

  switch (division) {
    case 1:
      M0 = 0;
      M1 = 0;
      M2 = 0;
      break;
    case 2:
      M0 = 1;
      M1 = 0;
      M2 = 0;
      break;
    case 4:
      M0 = 0;
      M1 = 1;
      M2 = 0;
      break;
    case 16:
      M0 = 1;
      M1 = 1;
      M2 = 0;
      break;
    case 32:
      M0 = 0;
      M1 = 0;
      M2 = 1;
      break;
    case 64:
      M0 = 1;
      M1 = 1;
      M2 = 1;
      break;
    default:
      return -4;
  }

  digitalWrite(M0_PIN, M0);
  digitalWrite(M1_PIN, M1);
  digitalWrite(M2_PIN, M2);

  return 0;
}

void reboot() {
  SCB_AIRCR = 0x05FA0004;
}



// Ping200XR

// Initialize the CRC-CCITT table
void crc_init() {
  for (int i = 0; i < 256; i++) {
    uint16_t crc = (uint16_t)(i << 8);
    for (int bitctr = 0; bitctr < 8; bitctr++) {
      crc = (crc << 1) ^ ((crc & 0x8000) ? CRC_POLYNOMIAL : 0);
    }
    Crc16Table[i] = crc & 0xFFFF;
  }
}

// Compute CRC-CCITT for a given data array
uint16_t crc_compute(const uint8_t* data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc = (Crc16Table[(crc >> 8) ^ data[i]] ^ (crc << 8)) & 0xFFFF;
  }
  return crc;
}

// Unescape payload
size_t unescape_payload(const uint8_t* payload, size_t length, uint8_t* unescaped) {
  size_t j = 0;
  for (size_t i = 0; i < length; i++) {
    if (payload[i] == ESCAPE_BYTE) {
      i++;
      unescaped[j++] = payload[i] ^ 0x20;
    } else {
      unescaped[j++] = payload[i];
    }
  }
  return j;
}

// Decode packet based on message ID
const char* decode_packet(const uint8_t* packet, size_t length) {
  if (length < 3) return "Invalid packet length";

  uint8_t msg_id = packet[0];
  uint8_t version = packet[1];
  // Serial.println(msg_id);
  switch (msg_id) {
    case 0x00:
      {
        // uint8_t gnss_valid = (packet[2] >> 7) & 0x01;
        // uint8_t maintenance_req = (packet[2] >> 6) & 0x01;
        // uint8_t ident_active = (packet[2] >> 5) & 0x01;
        // uint8_t initialized = packet[2] & 0x01;

        parseHeartbeatMessage(&heartbeatMessage, packet);

        // printf("Message ID: %d\n", msg_id);
        // printf("GNSS Valid: %d\n", gnss_valid);
        // printf("Maintenance Required: %d\n", maintenance_req);
        // printf("IDENT Active: %d\n", ident_active);
        // printf("Device Initialized: %d\n", initialized);

        return "Heartbeat message decoded";
      }

    case 0x0A:
      {
        parseOwnshipReport(&ownshipReport, packet, length);
        // Serial.println(ownshipReport.flightIdentification);
        return "Ownship report";
      }

    case 0x0B:
      {
        parseGeometricAltitude(&geometricAltitude, packet);
        // Serial.println()
        return "Geometric Altitude";
      }

    case 0x2E:
      {
        parseGNSSData(&gnssData, packet);
        Serial.println("here");
        return "GNSS data";
      }

    case 0x2F:
      {
        parseTransponderStatus(&transponderStatus, packet);
        return "Transponder Status";
      }

    case 0x28:
      {
        parseBarometerSensor(&barometerSensor, packet, length);
        return "Barometer Sensor";
      }

    default:

      break;
  }

  // Add handling for additional message types as needed...
  return "Unknown message type";
}

// Process packet function
const char* process_packet(const uint8_t* data, size_t length) {
  if (data[0] != FLAG_BYTE || data[length - 1] != FLAG_BYTE) {
    return "Invalid frame flags";
  }

  // Remove framing flags
  uint8_t packet[length - 2];
  memcpy(packet, data + 1, length - 2);

  // Unescape payload
  uint8_t unescaped_packet[length - 2];
  size_t unescaped_len = unescape_payload(packet, length - 2, unescaped_packet);

  // Verify CRC
  uint16_t crc_received = (unescaped_packet[unescaped_len - 2] << 8) | unescaped_packet[unescaped_len - 1];
  uint16_t crc_computed = crc_compute(unescaped_packet, unescaped_len - 2);

  // if (crc_computed != crc_received) {
  //     return "CRC check failed";
  // }

  // Decode payload
  return decode_packet(unescaped_packet, unescaped_len - 2);
}

// Test the script with a sample packet
void test_sample_packet() {
  crc_init();  // Initialize CRC table

  // Example sample packet
  uint8_t sample_packet[] = { 0x7E, 0x80, 0x60, 0x18, 0x80, 0x80, 0x80, 0xFE, 0xFF, 0xFF, 0xFF, 0xFB, 0x30, 0x18, 0x7B, 0xFC, 0x7E };
  const char* result = process_packet(sample_packet, sizeof(sample_packet));
  printf("%s\n", result);
}

void printpacket(uint8_t* packet, int length) {
  // Here you can process the packet stored in `packetBuffer`
  Serial.print("Received packet: ");
  for (int i = 0; i < length; i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  uint8_t data[length + 1];
  memcpy(data, &packet, length * sizeof(packet[0]));
  data[length] = 0x7E;
  const char* result = process_packet(*data, sizeof(data));

  for (uint8_t item : data) { Serial.print(item, HEX); };
  Serial.println();
}

void parseHeartbeatMessage(HeartbeatMessage* heartbeat, const uint8_t* data) {
  heartbeat->gnss_valid = (data[2] >> 7) & 0x01;
  heartbeat->maintenance_req = (data[2] >> 6) & 0x01;
  heartbeat->ident_active = (data[2] >> 5) & 0x01;
  heartbeat->initialized = data[2] & 0x01;
}

void parseOwnshipReport(OwnshipReport* report, const uint8_t* data, size_t length) {
  report->messageID = data[0];
  report->trafficAlertStatus = (data[1] & 0xF0) >> 4;
  int32_t latitudeRaw = (int32_t)((data[5] << 16) | (data[6] << 8) | data[7]);
  if (latitudeRaw & 0x800000) {  // Check 24th bit for negative sign
    latitudeRaw |= 0xFF000000;   // Properly sign extend to 32-bit
  }
  report->latitude = latitudeRaw * (180.0f / (1 << 23));

  int32_t longitudeRaw = (int32_t)((data[8] << 16) | (data[9] << 8) | data[10]);
  if (longitudeRaw & 0x800000) {  // Check 24th bit for negative sign
    longitudeRaw |= 0xFF000000;   // Properly sign extend to 32-bit
  }
  report->longitude = longitudeRaw * (180.0f / (1 << 23));

  report->altitude = (((data[11] << 8) | data[12]) >> 4) * 25 - 1000;
  report->miscellaneousIndicators = data[12] & 0x0F;
  report->NIC = (data[13] & 0xF0) >> 4;
  report->NACp = data[13] & 0x0F;
  report->horizontalVelocity = (data[14] << 4) | (data[15] >> 4);
  report->verticalVelocity = ((data[15] & 0x0F) << 8) | data[16];
  report->trackHeading = data[17];
  report->emitterCategory = data[18];
  memcpy(report->flightIdentification, &data[19], 6);
  report->flightIdentification[6] = '\0';
}


void parseGeometricAltitude(GeometricAltitude* altitude, const uint8_t* data) {
  altitude->messageID = data[0];
  altitude->geometricAltitude = ((int16_t)((data[1] << 8) | data[2])) * 5;
}

void parseGNSSData(GNSSData* gnss, const uint8_t* data) {
  gnss->messageID = data[0];
  gnss->messageVersion = data[1];
  gnss->utcTime = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
  gnss->latitude = ((int32_t)((data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9])) / 1e7;
  gnss->longitude = ((int32_t)((data[10] << 24) | (data[11] << 16) | (data[12] << 8) | data[13])) / 1e7;
  gnss->altitude = ((int32_t)((data[14] << 24) | (data[15] << 16) | (data[16] << 8) | data[17])) / 1e3;
  gnss->hpl = ((data[18] << 24) | (data[19] << 16) | (data[20] << 8) | data[21]) / 1e3;
  gnss->vpl = ((data[22] << 24) | (data[23] << 16) | (data[24] << 8) | data[25]) / 1e2;
  gnss->hfom = ((data[26] << 24) | (data[27] << 16) | (data[28] << 8) | data[29]) / 1e3;
  gnss->vfom = ((data[30] << 8) | data[31]) / 1e2;
  gnss->hvfom = ((data[32] << 8) | data[33]) / 1e3;
  gnss->vvfom = ((data[34] << 8) | data[35]) / 1e3;
  gnss->gnssVerticalSpeed = ((int16_t)((data[36] << 8) | data[37])) / 1e2;
  gnss->northSouthVelocity = ((int16_t)((data[38] << 8) | data[39])) / 1e1;
  gnss->eastWestVelocity = ((int16_t)((data[40] << 8) | data[41])) / 1e1;
}

void parseTransponderStatus(TransponderStatus* status, const uint8_t* data) {
  status->messageID = data[0];
  status->messageVersion = data[1];
  status->txEnabled = (data[2] & 0x80) >> 7;
  status->identButtonActive = (data[2] & 0x08) >> 3;
}

void parseBarometerSensor(BarometerSensor* sensor, const uint8_t* data, size_t length) {

  if (length != 12) {
    // Serial.println("Invalid data length for Barometer Sensor message. Expected 12 bytes.");
    return sensor;
  }

  sensor->messageID = data[0];
  sensor->sensorType = data[1];
  sensor->barometricPressure = ((uint32_t)(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5])) / 100.0;
  sensor->barometricPressureAltitude = (int32_t)(data[6] << 24 | data[7] << 16 | data[8] << 8 | data[9]);
  sensor->barometricSensorTemperature = ((int16_t)(data[10] << 8 | data[11])) / 100.0;

  if (sensor->barometricPressureAltitude == 0xFFFFFFFF)
    sensor->barometricPressureAltitude = -1;  // Indicates "Invalid"
  if (sensor->barometricSensorTemperature == (int16_t)(0xFFFF / 100))
    sensor->barometricSensorTemperature = -1.0;  // Indicates "Invalid"
}

void PingHandler() {
  while (1) {
    while (PingSerial.available() > 0) {
      // foundpacketindex = 0;
      byte incomingByte = PingSerial.read();

      // Check for start of packet
      if (incomingByte == 0x7E) {
        // If we're already capturing a packet, this is the end marker
        if (capturing && packetIndex > 0) {
          // End of packet - add the final 0x7E byte
          packetBuffer[packetIndex++] = 0x7E;

          // Copy the detected packet to foundPacket
          memcpy(foundPacket, packetBuffer, packetIndex);
          // foundpacketindex = packetIndex;
          // Print the found packet for debugging
          // Serial.print("Detected Packet: ");
          // for (int i = 0; i < packetIndex; i++) {
          //     Serial.print(foundPacket[i], HEX);
          //     Serial.print(" ");
          // }
          // Serial.println();

          // parse
          const char* result = process_packet(foundPacket, packetIndex);
          // Serial.println(result);


          // Reset for the next packet
          packetIndex = 0;
          capturing = false;
        } else {
          // Start of a new packet
          capturing = true;
          packetIndex = 0;
          packetBuffer[packetIndex++] = 0x7E;
        }
      } else if (capturing) {
        // Continue storing bytes until maxPacketSize is reached
        if (packetIndex < maxPacketSize) {
          packetBuffer[packetIndex++] = incomingByte;
        } else {
          // If packet exceeds buffer size, reset
          capturing = false;
          packetIndex = 0;
        }
      }
    }
  }
}

void test() {
  while (1) {
    Serial.print(ownshipReport.flightIdentification);
    Serial.print(", ");
    Serial.print(heartbeatMessage.gnss_valid);
    Serial.print(", ");
    Serial.print(ownshipReport.latitude);
    Serial.print(", ");
    Serial.print(ownshipReport.longitude);
    Serial.print(", ");
    Serial.print(barometerSensor.barometricPressure);
    Serial.println();
    delay(200);
  }
}

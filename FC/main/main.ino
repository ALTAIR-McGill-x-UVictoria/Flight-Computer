//Flight Computer Main

// #include <RadioLib.h>
#include <ArduinoQueue.h>
#include "Waveshare_10Dof-D.h"
// #include "RadioLogic.h"
#include <SPI.h>
#include <RH_RF95.h>
#include "Waveshare_10Dof-D.h"
#include <SD.h>
#include <Bonezegei_DRV8825.h>

//Callsign
#define CALLSIGN "VA2ETD"

//Radio pin definitions
#define RFM95_RST 20  //5
#define RFM95_CS 10
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
#define MOTOR_BATTERY_PIN 25
#define HEATING_BATTERY_PIN 22
float RH2 = 10000;
float RH1 = 2800;
float R2 = 3520;
float R1 = 10000;
float voltage;

//Functionality enable definitions
#define RX_ENABLE 1
#define DAQ_ENABLE 1
#define SD_ENABLE 1
#define LED_ENABLE 1
#define STEPPER_ENABLE 0
#define ACTUATOR_ENABLE 1
#define HEATING_ENABLE 1
#define DAQ_DEBUG 0
#define SD_DEBUG 0

//Radio loop timer (fastest transmission speed)
#define LOOP_TIMER 500
#define FLIGHT_MODE_TIMER 200
#define FLIGHT_MODE_COUNT 5

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


#define DIR_PIN 2
#define STEP_PIN 3
#define SLEEP_PIN 4
#define RESET_PIN 5
#define FAULT_PIN 6

#define M0_PIN 7
#define M1_PIN 8
#define M2_PIN 9

#define STEPS_PER_REV 200

#define NUM_LEDS 3

#define CW 0
#define CCW 1

#define USER 0
#define SYS 1





//end of definitions





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

float LEDtimerInput = 0;

char* packetToSend;
char* packetForm;

int toggleLongPacket = 0;
int toggleFlightMode = 0;
int transmissionCounter = 0;
int timeout = 1500;
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
#define ACTUATOR_PIN 8
bool heatingStatus = 0;
#define HEATING_PIN 39

#define LED_PIN 13
bool led_state = false;

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
#endif

#if SD_ENABLE
  SDSetup();
  // SDNewFile();
#endif

#if LED_ENABLE
  pinMode(PWM_LED1, OUTPUT);
  pinMode(PWM_LED2, OUTPUT);
  pinMode(PWM_LED3, OUTPUT);
#endif

  pinMode(7,OUTPUT);


  // pinMode(ANALOG_IN_PIN, INPUT); //voltage sensor

#if SD_ENABLE
  SDWrite("System init complete");
#endif

#if ENABLE_SERIAL
  Serial.println("Init complete");
#endif

#if STEPPER_ENABLE
  stepperSetup();
#endif

#if ACTUATOR_ENABLE
  pinMode(ACTUATOR_PIN, OUTPUT);
#endif

#if HEATING_ENABLE
  pinMode(HEATING_PIN, OUTPUT);
#endif

}

void loop() {

  if (DAQ_DEBUG) {
    fullSensorLoop();
  }


  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  payload_yaw = stAngles.fYaw;


  #if SD_ENABLE
  if (enableSDWrite == 1) {
    SDWrite(logger());
  }
  #endif

  #if RX_ENABLE
    commandid = String(rand()).substring(0,4);

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

  #if LED_ENABLE
    LEDhandler();

    //dont forget to remove
    
  #endif

  #if ACTUATOR_ENABLE
    actuatorHandler();
  #endif
  
  #if HEATING_ENABLE
    heatingHandler();
  #endif

  #if STEPPER_ENABLE
    stepperHandler();
  #endif
}




/*
=========================
Function definitions
=========================
*/



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
  // Serial.println("LoRa radio init OK!");

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

void radioTx(char radiopacket[100]) {
  
  rf95.send((uint8_t*)radiopacket, toggleLongPacket ? 90 : 20);

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

void fullSensorLoop() {

  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

  Serial.println();
  Serial.println("/-------------------------------------------------------------/");
  Serial.print("Roll : ");
  Serial.print(stAngles.fRoll);
  Serial.print("    Pitch : ");
  Serial.print(stAngles.fPitch);
  Serial.print("    Yaw : ");
  Serial.print(stAngles.fYaw);
  Serial.println();
  Serial.print("Acceleration: X : ");
  Serial.print(stAccelRawData.s16X);
  Serial.print("    Acceleration: Y : ");
  Serial.print(stAccelRawData.s16Y);
  Serial.print("    Acceleration: Z : ");
  Serial.print(stAccelRawData.s16Z);
  Serial.println();
  Serial.print("Gyroscope: X : ");
  Serial.print(stGyroRawData.s16X);
  Serial.print("       Gyroscope: Y : ");
  Serial.print(stGyroRawData.s16Y);
  Serial.print("       Gyroscope: Z : ");
  Serial.print(stGyroRawData.s16Z);
  Serial.println();
  Serial.print("Magnetic: X : ");
  Serial.print(stMagnRawData.s16X);
  Serial.print("      Magnetic: Y : ");
  Serial.print(stMagnRawData.s16Y);
  Serial.print("      Magnetic: Z : ");
  Serial.print(stMagnRawData.s16Z);
  Serial.println();
  Serial.print("Pressure : ");
  Serial.print((float)s32PressureVal / 100);
  Serial.print("     Altitude : ");
  Serial.print((float)s32AltitudeVal / 100);
  Serial.println();
  Serial.print("Temperature : ");
  Serial.print((float)s32TemperatureVal / 100);
  Serial.println();
  // delay(100);
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
  // Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    // Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }

  // Serial.println("Card initialized.");
  int i = 0;
  while(SD.exists(("datalog_" + String(i) + ".txt").c_str())){
    i++;
  }
  currentFilePath = "datalog_" + String(i) + ".txt";
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

void SDNewFile(){
  int i = 0;
  String loghead = "datalog_";
  while(SD.exists((loghead + String(i) + ".txt").c_str())){
    i++;
  }
  currentFilePath = loghead + String(i++) + ".txt";

}

String logger(){
  String log = "";
  log = log + ":" + reception_confirm + "," + rf95.lastRssi() + "," + rf95.lastSNR() + "," + battery_voltage("main") + "," + battery_voltage("motor") + "," + battery_voltage("main")+ stAngles.fPitch + "," + stAngles.fRoll + "," + stAngles.fYaw + "," + stAccelRawData.s16X + "," + stAccelRawData.s16Y + "," + stAccelRawData.s16Z + "," + (float)s32PressureVal / 100 + "," + (float)s32AltitudeVal / 100 + "," + (float)s32TemperatureVal / 100 + "," + led1Status + led2Status + led3Status + "," + ledIntensity + "," + enableSDWrite;
  return log;
}

char* formRadioPacket(bool enable_long, String cmdid) {  //includes DAQ

  //FOR FUTURE, FORM AS STRUCT AND TRANSMIT WITHOUT COMMAS 

  String packet = "";
  
  digitalWrite(7,reception_confirm == 1 ? HIGH : LOW);
  
  if (enable_long == 1) {

    packet = packet + cmdid + ":" + reception_confirm + "," + rf95.lastRssi() + "," + rf95.lastSNR() + "," + battery_voltage("main") + "," + stAngles.fPitch + ","
    + stAngles.fRoll + "," + stAngles.fYaw + "," + stAccelRawData.s16X + "," + stAccelRawData.s16Y + "," + stAccelRawData.s16Z + "," + (float)s32PressureVal / 100 + ","
    + (float)s32AltitudeVal / 100 + "," + (float)s32TemperatureVal / 100 + "," + led1Status + led2Status + led3Status + "," + ledIntensity + "," + enableSDWrite + "," + heatingStatus + "," + actuatorStatus + "," 
    + battery_voltage("motor") + "," + battery_voltage("heating");

    reception_confirm = 0;

    return packet.c_str();
  }

  //default small packet
  
 
  packet = packet + cmdid + ":" + reception_confirm + "," + rf95.lastRssi() + "," + rf95.lastSNR() + "," + battery_voltage("main");

  
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

  if(NULL != strtokIndx)
  {
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

  if(commandid != recvdCommandid){
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

      reception_confirm = 1;
      break;

    case 8:
      //sdstop
      enableSDWrite = 0;

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
      reception_confirm = 1;
      break;

    case 20:
      actuatorStatus = 0;
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
  
  float resistorRatio = 0;

  if (battery_select == "main" || battery_select == "heating"){
    resistorRatio = R2 / (R2 + R1);
  }
  else if(battery_select == "heating"){
    resistorRatio = RH2 / (RH2 + RH1);
  }

  float conversionFactor = 3.3 / (1024 * resistorRatio);

  float mainVoltage = (float)(analogRead(MAIN_BATTERY_PIN) * conversionFactor);
  float motorVoltage = (float)(analogRead(MOTOR_BATTERY_PIN) * conversionFactor);
  float heatingVoltage = (float)(analogRead(HEATING_BATTERY_PIN) * conversionFactor);

  if (battery_select == "main"){return mainVoltage;}
  else if(battery_select == "motor"){return motorVoltage;}
  else if(battery_select == "heating"){return heatingVoltage;}
  else{return 0.0;}
  
}

void LEDhandler() {

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

void actuatorHandler(){
  digitalWrite(ACTUATOR_PIN, actuatorStatus == 1 ? HIGH : LOW);
}

void stepperSetup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(FAULT_PIN, INPUT);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  // pinMode(LED_PIN, OUTPUT);


  //Disables sleep and reset
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(SLEEP_PIN, HIGH);

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

void heatingHandler(){
  heatingStatus = (float)s32TemperatureVal/100 < 15.0 ? 1 : 0;
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
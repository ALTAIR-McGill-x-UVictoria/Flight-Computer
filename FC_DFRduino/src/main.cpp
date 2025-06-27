//Flight Computer Main

#include <ArduinoQueue.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Arduino.h>

//Callsign
#define CALLSIGN "VA2ETD"
#define SHOW_CALLSIGN 0 //will show callsign in serial monitor

//Radio debugging without ground station
#define DEBUG_TX 0
#define LOOP_TIMER 1000 

//Radio pin definitions
#define RFM95_RST 7
#define RFM95_CS 10
#define RFM95_INT 2

//LoRa parameters definitions
#define RF95_FREQ 915.0
#define SF 8
#define BW 125000
#define TX_POWER 20

#define TIMEOUT 1500 // Timeout for radio receive operations in milliseconds

// Add at the top with other global variables
unsigned long lastTransmitTime = 0;
unsigned long transmitTimer = 0;
unsigned long currentMillis = 0;

// Singleton instances
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Buffer for receiving commands
#define CMD_BUFSIZE 12
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

String lastPacketReceived = "NO UPLINK"; // packet reception shows "ACTIVE" or "NO UPLINK"

// Command reception variables
int commandReceived = 0;
int lastCommandID = 0;


void radioSetup() {
  Serial.println("Starting radio setup...");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  delay(100);

  // Check SPI by reading RFM95 version register
  SPI.begin();
  // Try reading RFM95 version register (should return 0x12)
  digitalWrite(RFM95_CS, LOW);
  SPI.transfer(0x42 & 0x7F); // Version register address (0x42) with read bit
  uint8_t version = SPI.transfer(0x00); // Read value
  digitalWrite(RFM95_CS, HIGH);
  
  Serial.print("SPI read version: 0x");
  Serial.println(version, HEX);
  
  if (version == 0x12) {
    Serial.println("SPI communication with RFM95 successful!");
  } else {
    Serial.println("SPI working but couldn't verify RFM95 (should return 0x12)");
  }

  // Add this at the end of radioSetup() function
  uint8_t modemConfig1, modemConfig2;
  // Fix spiRead calls - they take only one parameter and return the value directly
  modemConfig1 = rf95.spiRead(RH_RF95_REG_1D_MODEM_CONFIG1);
  modemConfig2 = rf95.spiRead(RH_RF95_REG_1E_MODEM_CONFIG2);
  Serial.print("ModemConfig1: 0x"); Serial.println(modemConfig1, HEX);
  Serial.print("ModemConfig2: 0x"); Serial.println(modemConfig2, HEX);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    delay(1000); // Add delay instead of infinite loop
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    delay(1000); // Add delay instead of infinite loop
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setSignalBandwidth(BW);
  rf95.setCodingRate4(5);
  rf95.setSpreadingFactor(SF);
  rf95.setTxPower(TX_POWER, false);
  rf95.setPayloadCRC(true); 
  
  Serial.println("Radio setup complete");

  // Debug SPI communication
  Serial.println("Checking all registers for debug:");
  for (uint8_t reg = 0; reg <= 0x50; reg += 0x10) {
    Serial.print("Regs 0x"); Serial.print(reg, HEX); 
    Serial.print("-0x"); Serial.print(reg+0xF, HEX); Serial.print(": ");
    for (uint8_t i = 0; i <= 0xF; i++) {
      digitalWrite(RFM95_CS, LOW);
      SPI.transfer((reg+i) & 0x7F); // Fixed parentheses - Read bit
      uint8_t val = SPI.transfer(0x00);
      digitalWrite(RFM95_CS, HIGH);
      Serial.print(val, HEX); Serial.print(" ");
    }
    Serial.println();
  }
}

// Generate telemetry packet to send to ground station
String generateTelemetryPacket() {  
  
  // Format: connection status,RSSI,SNR,timestamp
  String packet = lastPacketReceived + ",";  // Connection status (0 or 1)
  packet += String(commandReceived) + ",";  // Reception status last command (0 or 1)
  packet += String(rf95.lastRssi()) + ",";  // RSSI of last received packet
  packet += String(rf95.lastSNR()) + ",";
  packet += String(millis() / 1000.0);
  
  return packet;
}

// Parse commands received from ground station
void parseGroundCommand(char* cmd) {
  char* strtokIndx;
  int cmdID = 0;
  float cmdValue = 0.0;
  
  // Format expected: cmdID,value
  strtokIndx = strtok(cmd, ",");
  if (strtokIndx != NULL) {
    cmdID = atoi(strtokIndx);
    lastCommandID = cmdID;
  }
  
  strtokIndx = strtok(NULL, ",");
  if (strtokIndx != NULL) {
    cmdValue = atof(strtokIndx);
  }
  
  commandReceived = cmdID; // Set command received flag
  // Process the command
  switch (cmdID) {
    case 0:
      break;
    case 1:
      // Ping command
      break;
    default:
      break;
  }
}

void radioTransmit() {
  // Step 1: Transmit telemetry data
  String telemetry = generateTelemetryPacket();
  
  Serial.print("Sending telemetry: ");
  Serial.println(telemetry);
  // Send the packet
  digitalWrite(LED_BUILTIN, HIGH);
  rf95.send((uint8_t*)telemetry.c_str(), telemetry.length());
  rf95.waitPacketSent();
  digitalWrite(LED_BUILTIN, LOW);
  
  // Step 2: Wait for response from ground station
  unsigned long waitStartTime = millis();
  while ((millis() - waitStartTime) < 1000) { // Wait up to 1 second for reply
    if (rf95.available()) {
      // Got a response
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        buf[len] = 0; // Null terminate
        lastPacketReceived = "ACTIVE"; // Set to active if we received a packet
        
        // Parse and execute the command
        parseGroundCommand((char*)buf);
        break;
      }
      lastPacketReceived = "NO UPLINK"; // No packet received
    }

    delay(10); // Small delay while waiting
  }
  
  // Reset command received flag for next transmission
  if (millis() - waitStartTime >= 1000) {
    commandReceived = 0;
  }
}

void setup() {
  // Setup serial
  Serial.begin(115200);
  delay(250);
  
  // Clear input buffer
  while(Serial.available()) {
    Serial.read();
  }
  
  // Setup onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Flight Computer start");
  radioSetup();
  
  Serial.println("Init OK");
  Serial.println("Ready:");
}

void loop() {
  currentMillis = millis();

  // Transmit data at regular intervals
  if (currentMillis - transmitTimer >= LOOP_TIMER) {
    radioTransmit();
    transmitTimer = currentMillis;
  }

}
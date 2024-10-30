#include <SD.h>
#include <string.h>
#include <TeensyThreads.h>

#define CRC_POLYNOMIAL 0x1021
#define FLAG_BYTE 0x7E
#define ESCAPE_BYTE 0x7D

uint16_t Crc16Table[256];


#define PingSerial Serial7
#define PingBaud 56700
// #define PingBaud 2400

const int maxPacketSize = 150;  // Define the maximum size for each packet (adjust as needed)
byte packetBuffer[maxPacketSize]; // Buffer to store the complete packet
byte foundPacket[maxPacketSize];  // Separate buffer to store the detected packet
int packetIndex = 0;              // Index to track position in packetBuffer
bool capturing = false;           // Flag to check if we are capturing a packet
int foundpacketindex = 0;

// struct definitions

struct HeartbeatMessage{
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


HeartbeatMessage heartbeatMessage;
OwnshipReport ownshipReport;
GeometricAltitude geometricAltitude;
GNSSData gnssData;
TransponderStatus transponderStatus;
BarometerSensor barometerSensor;

void setup() {
  // put your setup code here, to run once:
  while (!Serial && (millis() < 6000));

  PingSerial.begin(PingBaud);

  crc_init();

  int a = threads.addThread(PingHandler);
  int b = threads.addThread(test);
  // threads.start()

  Serial.println("Setup complete");

  

}

void loop() {
    
}


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
uint16_t crc_compute(const uint8_t *data, size_t length) {
    uint16_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc = (Crc16Table[(crc >> 8) ^ data[i]] ^ (crc << 8)) & 0xFFFF;
    }
    return crc;
}

// Unescape payload
size_t unescape_payload(const uint8_t *payload, size_t length, uint8_t *unescaped) {
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
const char* decode_packet(const uint8_t *packet, size_t length) {
    if (length < 3) return "Invalid packet length";

    uint8_t msg_id = packet[0];
    uint8_t version = packet[1];
    // Serial.println(msg_id);
    switch (msg_id) {
      // Serial.println(msg_id);
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

      case 10:
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

      case 46:
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
const char* process_packet(const uint8_t *data, size_t length) {
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
    // Serial.println(String(*data), HEX);
    // Decode payload
    return decode_packet(unescaped_packet, unescaped_len - 2);
}

// Test the script with a sample packet
void test_sample_packet() {
    crc_init(); // Initialize CRC table

    // Example sample packet
    uint8_t sample_packet[] = {0x7E, 0x80, 0x60, 0x18, 0x80, 0x80, 0x80, 0xFE, 0xFF, 0xFF, 0xFF, 0xFB, 0x30, 0x18, 0x7B, 0xFC, 0x7E};
    const char *result = process_packet(sample_packet, sizeof(sample_packet));
    printf("%s\n", result);
}

void printpacket(uint8_t *packet, int length) {
    // Here you can process the packet stored in `packetBuffer`
    Serial.print("Received packet: ");
    for (int i = 0; i < length; i++) {
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    uint8_t data[length + 1];
    memcpy(data, &packet, length*sizeof(packet[0]));
    data[length] = 0x7E;
    const char *result = process_packet(*data, sizeof(data));

    for (uint8_t item: data){Serial.print(item,HEX);};Serial.println();
}

void parseHeartbeatMessage(HeartbeatMessage* heartbeat, const uint8_t* data){
  heartbeat->gnss_valid = (data[2] >> 7) & 0x01;
  heartbeat->maintenance_req = (data[2] >> 6) & 0x01;
  heartbeat->ident_active = (data[2] >> 5) & 0x01;
  heartbeat->initialized = data[2] & 0x01;
}

void parseOwnshipReport(OwnshipReport* report, const uint8_t* data, size_t length) {
    // if (length != 28) {
    //     Serial.println("Invalid data length for Ownship Report. Expected 28 bytes.");
    //     return;
    // }
    
    report->messageID = data[0];
    report->trafficAlertStatus = (data[1] & 0xF0) >> 4;
    // report->addressType = data[1] & 0x0F;
    // snprintf(report->participantAddress, sizeof(report->participantAddress), "%02X%02X%02X", data[2], data[3], data[4]);
    // report->participantAddress = (data[2] << 16) | (data[3] << 8) | data[4];
    int32_t latitudeRaw = (int32_t)((data[5] << 16) | (data[6] << 8) | data[7]);
    if (latitudeRaw & 0x800000) { // Check 24th bit for negative sign
      latitudeRaw |= 0xFF000000; // Properly sign extend to 32-bit
    }
    report->latitude = latitudeRaw * (180.0f / (1 << 23));

    int32_t longitudeRaw = (int32_t)((data[8] << 16) | (data[9] << 8) | data[10]);
    if (longitudeRaw & 0x800000) { // Check 24th bit for negative sign
      longitudeRaw |= 0xFF000000; // Properly sign extend to 32-bit
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
        Serial.println("Invalid data length for Barometer Sensor message. Expected 12 bytes.");
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

void PingHandler(){
  while(1){
    unsigned int incomingByte = 0;
    while (PingSerial.available() > 0) {
      // foundpacketindex = 0;
          // byte incomingByte = PingSerial.read();
          incomingByte = PingSerial.read();
          // Serial.print(String(incomingByte,HEX)); Serial.print("-");
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
                  const char *result = process_packet(foundPacket, packetIndex);
                  // Serial.println(result);


                  // Reset for the next packet
                  packetIndex = 0;
                  capturing = false;
              } 
              else {
                  // Start of a new packet
                  capturing = true;
                  packetIndex = 0;
                  packetBuffer[packetIndex++] = 0x7E;
              }
          }
          else if (capturing) {
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

void test(){
  while(1){
    Serial.print(ownshipReport.flightIdentification);
    Serial.print(", ");
    Serial.print(heartbeatMessage.gnss_valid);
    Serial.print(", ");
    Serial.print(ownshipReport.latitude,DEC);
    Serial.print(", ");
    Serial.print(ownshipReport.longitude,DEC);
    // Serial.print(", ");
    // Serial.print(barometerSensor.barometricPressure);
    Serial.println();
    delay(200);
  }
}


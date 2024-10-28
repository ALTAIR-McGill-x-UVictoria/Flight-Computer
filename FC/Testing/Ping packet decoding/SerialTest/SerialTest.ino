// #include "uart.h"
#include <SD.h>
#include <string.h>

#define CRC_POLYNOMIAL 0x1021
#define FLAG_BYTE 0x7E
#define ESCAPE_BYTE 0x7D

uint16_t Crc16Table[256];


#define PingSerial Serial3
#define PingBaud 56700

const int maxPacketSize = 64;  // Define the maximum size for each packet (adjust as needed)
byte packetBuffer[maxPacketSize]; // Buffer to store the complete packet
byte foundPacket[maxPacketSize];  // Separate buffer to store the detected packet
int packetIndex = 0;              // Index to track position in packetBuffer
bool capturing = false;           // Flag to check if we are capturing a packet
int foundpacketindex = 0;

void setup() {
  // put your setup code here, to run once:
  while (!Serial && (millis() < 6000));

  PingSerial.begin(PingBaud);

  crc_init();

  Serial.println("Setup complete");


}

void loop() {
  
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
                foundpacketindex = packetIndex;
                // Print the found packet for debugging
                Serial.print("Detected Packet: ");
                for (int i = 0; i < packetIndex; i++) {
                    Serial.print(foundPacket[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();

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

  for (int i = 0; i <= foundpacketindex; i++){
    Serial.print(foundPacket[i],HEX);
  }
  Serial.println();
  
  

  // delay(100);
  
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

    // Example: Handling Heartbeat message (Msg ID = 0x00)
    if (msg_id == 0x00) {
        uint8_t gnss_valid = (packet[2] >> 7) & 0x01;
        uint8_t maintenance_req = (packet[2] >> 6) & 0x01;
        uint8_t ident_active = (packet[2] >> 5) & 0x01;
        uint8_t initialized = packet[2] & 0x01;

        printf("Message ID: %d\n", msg_id);
        printf("GNSS Valid: %d\n", gnss_valid);
        printf("Maintenance Required: %d\n", maintenance_req);
        printf("IDENT Active: %d\n", ident_active);
        printf("Device Initialized: %d\n", initialized);

        return "Heartbeat message decoded";
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

    if (crc_computed != crc_received) {
        return "CRC check failed";
    }

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


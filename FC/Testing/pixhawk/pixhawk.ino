// #include <mavlink.h>
#include <MAVLink.h>

// Define Serial Ports
#define SERIAL_BAUD 57600  // Adjust as per your telemetry settings

// HardwareSerial &pixhawkSerial = Serial1;  // Use Serial1 for Pixhawk connection (Mega, Due, ESP32)
#define pixhawkSerial Serial1

// MAVLink Communication Variables
uint8_t system_id = 255;   // ID of the Arduino
uint8_t component_id = 1;  // ID of the component (generic)
uint8_t target_system = 1; // ID of the Pixhawk
uint8_t target_component = 1;

// Timer variables for requesting data
unsigned long lastRequestTime = 0;
const int requestInterval = 1000; // Request data every second

void setup() {
    Serial.begin(115200); // Debugging Serial Monitor
    pixhawkSerial.begin(SERIAL_BAUD); // Connect to Pixhawk
    Serial.println("Starting MAVLink communication...");
}

void requestData(uint8_t message_id) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_request_data_stream_pack(
        system_id, component_id, &msg, 
        target_system, target_component, 
        message_id, 1, 1
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    pixhawkSerial.write(buf, len);
}

void readMavlinkMessages() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (pixhawkSerial.available()) {
        uint8_t c = pixhawkSerial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    Serial.print("Roll: "); Serial.print(attitude.roll);
                    Serial.print(" Pitch: "); Serial.print(attitude.pitch);
                    Serial.print(" Yaw: "); Serial.println(attitude.yaw);
                    break;
                }
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t gps;
                    mavlink_msg_global_position_int_decode(&msg, &gps);
                    Serial.print("Lat: "); Serial.print(gps.lat / 1E7);
                    Serial.print(" Lon: "); Serial.print(gps.lon / 1E7);
                    Serial.print(" Alt: "); Serial.println(gps.alt / 1000.0);
                    break;
                }
                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t sys_status;
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
                    Serial.print("Battery: "); Serial.print(sys_status.voltage_battery / 1000.0);
                    Serial.print("V CPU Load: "); Serial.println(sys_status.load / 10.0);
                    break;
                }
                default:
                    Serial.print("Received MAVLink ID: "); Serial.println(msg.msgid);
                    break;
            }
        }
    }
}

void loop() {
    unsigned long now = millis();

    if (now - lastRequestTime > requestInterval) {
        requestData(MAVLINK_MSG_ID_ATTITUDE);  // Request attitude data
        requestData(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);  // Request GPS data
        requestData(MAVLINK_MSG_ID_SYS_STATUS);  // Request system status
        lastRequestTime = now;
    }

    readMavlinkMessages();
}

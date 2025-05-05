/*
 UDPReceiveMavlinkMessages

 This sketch receives UDP Mavlink Messages and has a few examples of parced messages. you can easily add whatever messages you want to parse to the SWITCH case

 Created 2/29/2024
 by Tristan Williams

 */

#include "mavlink.h"
#include <Ethernet.h>
#include <EthernetUdp.h>

//rates from Pixhawk Mavlink Messages
double yaw_rate;
double roll_rate;
double pitch_rate;

//attitude from Pixhawk Mavlink Messages
double yaw;
double roll;
double pitch;

//enable to print status info to SERIAL
bool DEBUG = true;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[2048];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  Ethernet.init(10);  // Most Arduino shields

  // start the Ethernet
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  // start UDP
  Udp.begin(localPort);
}

void loop() {
  
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();

  if (packetSize) {

    IPAddress remote = Udp.remoteIP();

    // read the packet into packetBuffer
    Udp.read(packetBuffer, 2048);

    mavlink_message_t message;
    mavlink_status_t status;

    for (int i = 0; i < packetSize; ++i) {


        if (mavlink_parse_char(MAVLINK_COMM_0, packetBuffer[i], &message, &status) == 1) {

           //use this to see what all Message IDs are coming across
           // Serial.print("Received message:");
           // Serial.println(message.msgid);

        switch(message.msgid) {
        
          case MAVLINK_MSG_ID_RAW_IMU:  // #105 highres IMU / Check scaled IMU for acutal value readings 
            {
            
              mavlink_raw_imu_t raw_imu;
              mavlink_msg_raw_imu_decode(&message, &raw_imu);
              
              roll_rate = raw_imu.xgyro;
              yaw_rate = raw_imu.zgyro;
              pitch_rate = raw_imu.ygyro;
              
              //DEBUG
              if (DEBUG){

                Serial.print(1);
                Serial.print(",");
                Serial.print(roll_rate);
                Serial.print(", ");
                Serial.print(pitch_rate);
                Serial.print(", ");
                Serial.print(yaw_rate);
                Serial.println("");    

              }
              break;
            }
            
          case MAVLINK_MSG_ID_ATTITUDE:  // #30 ATTITUDE
            {
              
              mavlink_attitude_t attitude;
              mavlink_msg_attitude_decode(&message, &attitude);

              roll = attitude.roll;  // rad (-pi..+pi)
              pitch = attitude.pitch;  // rad (-pi..+pi)
              yaw = attitude.yaw;
              
              //DEBUG
              if(DEBUG){

                Serial.print(roll);
                Serial.print(", ");
                Serial.print(pitch);
                Serial.print(", ");
                Serial.print(yaw);
                Serial.println("");      

              }      
              break;
            }
        }
      }
    }

    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
  delay(10);
}


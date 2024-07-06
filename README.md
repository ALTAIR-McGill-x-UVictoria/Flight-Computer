# Flight-Computer

Radio Packet Format:

- Full: ||RSSI:xxx,SNR:xx,Roll:xxxx.xx,Pitch:xxxx.xx,Yaw:xxxx.xx,Pressure:xxxx.xx,Temperature:xxxx.xx,Altitude:xxxx.xx,LED:xx.x||
- Data sizes:
  - Roll, pitch, yaw: float (32 bits, 4 bytes)
  - Pressure, temperature, altitude: int32_t (32 bits, 4 bytes)

Radio Commands:
0. none
1. ping: pong
2. ledon [value]: LED on at value
3. ledoff: LED off
4. ds [angle]: set angle of driver
5. ds: Start daq write to SD
6. de: Stop daq write to SD

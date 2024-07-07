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
4. ledbright [value]: Set the bright of the LEDs. 
5. dangle [angle]: set angle of driver
6. sdwrite: Start daq write to SD
7. sdstop: Stop daq write to SD


Radio logic loop:
1. FC starts in transmit mode, GS starts in receive mode
2. FC transmits one (or more) packet(s), flags last packet with set to receive, switch to receive and waits x ms
3. GS receives packet with flag, switch to transmit send command then switch back to receive
4. If FC receives packet confirmation (code >= 0), instantly switch to transmit (back to step 2), else if no packet reception for duration of wait switch back to step 2

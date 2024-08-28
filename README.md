Flight-Computer

FC Radio Commands (ascending by code number):

0. none
1. ping: pong
2. led1 [value]: Toggle LED #1 
3. led2 [value]: Toggle LED #2
4. led3 [value]: Toggle LED #3
5. ledoff: All LEDs OFF
6. dangle [angle]: set angle of driver (relative to last zero angle)
7. sdwrite: Start DAQ write to SD
8. sdstop: Stop DAQ write to SD
9. sdclear: Delete the all current data written on SD
10. ledblink [time]: Enable LED blinking at [time] ms intervals
11. ledbright [value]: Set the brightness of the LEDs
12. togglelong: Toggle the full flight computer packet transmission
13. zeromotor: Sets the current motor angle as zero degrees
14. stepperspeed [value]: Sets the motor step speed (default 2000)
15. togglestab: Toggles inertial stabilization from IMU angle
16. toggleflightmode: Toggles telemetry flight mode (1 Rx/5 Tx vs. 1 Rx/Tx)
17. setradiotimeout [time]: Sets FC radio 'wait' timer (default 1000 ms)
18. resetfc: Flight Computer soft reset

Local Ground Station Commands:

- toggleparsing: Toggles parsing of flight computer packet in serial monitor
- clearq: Clears the current command queue



Code definitions (non exhaustive):
At the top of the code file (.ino), there are definitions for pins and functionalities. Listed here are the ones which can be changed. 

- CALLSIGN: the current operator's callsign (can keep Ben's advanced license callsign)
- LoRa parameters:
  - RF95_FREQ: the frequency to be used (keep at 433.0)
  - SF: spreading factor (optimal: 8, longer range, slower data: 12)
  - BW: bandwidth (optimal: 125000, longer range, slower data: 62000)
  - TX_POWER: transceiver output power, keep at 20 dBm (highest)

- Functionalities (0: disabled, 1: enabled):
  - RX_ENABLE: enables radio transceiver on this device
  - DAQ_ENABLE: enables IMU
  - SD_ENABLE: enables SD
  - LED_ENABLE: enables LEDs (warning: if LED power is plugged in, the LEDs will flash very bright when the Teensy is powered on)
  - ENABLE_SERIAL: enables printing packets to the serial moitor, useful for debugging. Note: must be disabled (0) when on battery power



FC order of operations:
0. (Initialize radio, IMU, SD)
1. Form radio packet (DAQ, LED status, SD status)
2. Write to SD
3. Ragio logic (transmit then receive)
4. Set LED PWM (0 to 255) 

Radio logic loop:
1. FC starts in transmit mode, GS starts in receive mode
2. FC transmits one packet, flags last packet with set to receive, switch to receive and waits 2500 ms (timeout)
3. GS receives packet with flag, switch to transmit send command then switch back to receive
4. If FC receives packet confirmation, instantly switch to transmit (skip timeout, back to step 2), else if no packet reception for duration of wait switch back to step 2

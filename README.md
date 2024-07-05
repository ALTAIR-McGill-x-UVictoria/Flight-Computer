# Flight-Computer

Radio Packet Format:

- Full: ||RSSI:xxx,SNR:xx,Roll:xxxx.xx,Pitch:xxxx.xx,Yaw:xxxx.xx,Pressure:xxxx.xx,Temperature:xxxx.xx,Altitude:xxxx.xx||
- Data sizes:
  - Roll, pitch, yaw: float (32 bits, 4 bytes)
  - Pressure, temperature, altitude: int32_t (32 bits, 4 bytes)

Radio Commands:
- pp: ping
- lh: LED high
- ll: LED low
- ds [angle]: set angle of driver
- ds: Start daq write to SD
- de: Stop daq write to SD

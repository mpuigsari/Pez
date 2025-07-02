#!/usr/bin/env python3
import serial, time, binascii

# configure ports
tx = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
rx = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

payload = bytes([0x01,0x02,0x03,0x04])
tx.write(payload)
time.sleep(0.1)
resp = rx.read(256)
print('Received hex:', binascii.hexlify(resp))

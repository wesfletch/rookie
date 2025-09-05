#!/usr/bin/env python3

import serial
import sys

port: str = "/dev/ttyACM0"
if len(sys.argv) > 1:
    port = sys.argv[1]


ser: serial.Serial = serial.Serial(
    port=port,
    baudrate=115200,
    timeout=1,
)

while True:
    data = ser.readline().decode("utf-8").rstrip()
    print(f"RX: {data}")

#!/usr/bin/env python3

import serial

# Device file for the GPS receiver
device = "/dev/ttyUSB0"

# Baud rate for the GPS receiver
baud_rate = 4800

# Factory reset NMEA message to send
nmea_message = '$PSRF101,0,0,0,0,0,0,12,8*1C\r\n'

# Open the serial port
with serial.Serial(device, baud_rate, timeout=1) as ser:
    # Send the NMEA message
    ser.write(nmea_message.encode('ascii'))
    print("Factory Reset NMEA message sent successfully.")

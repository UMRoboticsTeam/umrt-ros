#!/usr/bin/env python3

import serial
import time

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

    # Continuously read from the serial port
    try:
        while True:
            # Read a line from the serial port
            response = ser.readline()

            # Print the response
            if response:
                print("Response from GPS receiver:", response)

            # Sleep for a short period to avoid high CPU usage
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Exit the loop on a keyboard interrupt (Ctrl+C)
        ser.close()
        print("Stopping serial read loop.")

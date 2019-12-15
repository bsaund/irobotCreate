#!/usr/bin/env python

import serial


def sendCommandRaw(command):
    global connection

    try:
        if connection is not None:
            connection.write(command)
        else:
            print("Unable to send command: Not connected")
    except serial.SerialException:
        print("Lost connection")
        connection = None

    print(" ".join([str(ord(c)) for c in command]))
    
    
def onConnect():
    global connection
    port = "/dev/ttyUSB0"

    try:
        connection = serial.Serial(port, baudrate=115200, timeout=1)
        print "Connected!"
    except serial.SerialException:
        print "Failed to connect"


if __name__ == "__main__":
    print("hello")
    ser = serial.Serial('/dev/ttyUSB0')
    print(ser.name)

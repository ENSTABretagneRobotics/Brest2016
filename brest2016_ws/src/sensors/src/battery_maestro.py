#!/usr/bin/env python

import serial


class battChecker:
    def __init__(self, port=0):
        ttyStr = '/dev/ttyACM' + str(port)
        self.usb = serial.Serial(ttyStr)
        self.PololuCmd = chr(0xaa) + chr(0xc)
        # state of each pin/battery
        self.state = [0] * 12

    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()

    # the position represents the voltage measured on the channel.
    # The inputs on channels 0-11 are analog: their values range from 0 to 1023
    # representing voltages from 0 to 5 V.
    # The inputs on channels 12-23 are digital:
    # their values are either exactly 0 or exactly 1023.
    def getState(self):
        for chan in range(12):
            cmd = self.PololuCmd + chr(0x10) + chr(chan)
            self.usb.write(cmd)
            lsb = ord(self.usb.read())
            msb = ord(self.usb.read())
            self.state[chan] = (msb << 8) + lsb
        return self.state

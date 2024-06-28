#!/usr/bin/python3

import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600)

time.sleep(2)
arduino.write(bytes('6', 'utf-8'))

import numpy as np

import struct
import time
import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=None) #linux

def writeFloat(f):
   ser.write(struct.pack('>i', int(f*1000)))
def readFloat():
   return float(struct.unpack('>i', ser.read(4))[0])/1000
def setPower(p1, p2, p3):
   ser.write('p')
   writeFloat(p1)
   writeFloat(p2)
   writeFloat(p3)
def setSpeed(s1, s2, s3):
   ser.write('s')
   writeFloat(s1)
   writeFloat(s2) 
   writeFloat(s3)
def setPID(motor, p, i, qpps): #use motor = 0 to set all motors
   ser.write('k')
   ser.write(str(motor))
   writeFloat(p)
   writeFloat(i)
   writeFloat(qpps)
def setT(period_ms, tau_ms):
   ser.write('t')
   writeFloat(period_ms)
   writeFloat(tau_ms)
def getSpeed():
   ser.write('v')
   return readFloat(), readFloat(), readFloat()
def getEncoderCount():
   ser.write('e')
   return readFloat(), readFloat(), readFloat()
def disengage():
   print "Disengaging!"
   ser.write('d')
   
pulsePerRotation = 4955 #Old motors
#pulsePerRotation = 116.2 #New motors

def init():
   # Set the PIDQ values for each motor. TODO: tune these values.
   # defense values
   # setPID(1, 3.5, 1.5, 100000)
   # setPID(2, 3.5, 1.5, 100000)
   # setPID(3, 3.5, 1.5, 100000) 
   
   setPID(1, 2.0, 3.0, 100000)
   setPID(2, 2.0, 3.0, 100000)
   setPID(3, 2.0, 3.0, 100000) 

   # Set tick period (triggers PID control) and velocity filter corner frequency
   setT(20, 50)

def setWheelVelocities(rps1, rps2, rps3):
   global pulsePerRotation
   # orientation is rps2, rps3, rps1
   setSpeed(rps2*pulsePerRotation, rps3*pulsePerRotation, rps1*pulsePerRotation)

def straightLine(rps):
   global pulsePerRotation
   setSpeed(rps*pulsePerRotation, 0, -rps*pulsePerRotation)

def stopMoving():
   setSpeed(0, 0, 0)

# need the for loop for the motors to run -Brett


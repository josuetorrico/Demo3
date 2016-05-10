#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*

# ******************************************************

# Chris Glomb
# Demo 3
# 4/24/16

# ******************************************************

import numpy as np
import sys
import cv2
import time
import math
from ctypes import *
import serial
import struct
import RPi.GPIO as GPIO

SET_POSITION = 0
GET_POSITION = 1

DETECT = 2
CENTER = 3

imgX = 160
imgY = 120

cx = imgX/2
cy = imgY/2
fov = 60 # in degrees

er = np.ones((4,4),np.uint8)
di = np.ones((2,2),np.uint8)

red_lowerLim = np.array((0.,50.,50.))
red_upperLim = np.array((10.,255.,255.))

blue_lowerLim = np.array((110.,50.,50.))
blue_upperLim = np.array((130.,255.,255.))

x0,y0,z0 = 0,0,73 # limb origin
L1 = 140 # lower segment length
L2 = L1 # upper segment length

cv2.namedWindow("Gripper View", cv2.WINDOW_AUTOSIZE)

GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)
GPIO.output(26, False)

class GRIPPER_POSITION(Structure):
    _pack_ = 1
    _fields_ = [("x", c_float), ("y", c_float), ("z", c_float)]
    
class SERVO_POSITION(Structure):
    _pack_ = 1
    _fields_ = [("yaw", c_float), ("pitch", c_float), ("elbow", c_float), ("gripper", c_int)]

ser1 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

servo = SERVO_POSITION()
servo.yaw = -math.pi
servo.pitch = -0.2
servo.elbow = 2
servo.gripper = 400

cap = cv2.VideoCapture(0)
#sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
#host ="192.168.1.18" # IP of device to display the image
#port = 5005
#buf =1024
#addr = (host,port)



def FK(servoVal):
	# Calculate end position in spherical coordinates
	theta = servo.yaw
	r = math.sqrt(2*(L1**2) - 2*(L1**2)*math.cos(servoVal.elbow))
	thi = (math.pi/2 - (math.pi - servoVal.elbow)/2) + servoVal.pitch
	
	gripper = GRIPPER_POSITION()
	
	# Convert spherical to Cartesian
	gripper.x = r*math.cos(thi)*math.cos(theta) + x0
	gripper.y = r*math.sin(thi)*math.sin(theta) + y0
	gripper.z = r*math.sin(thi) + z0

	if abs(gripper.x) < 0.01:
		gripper.x = 0
	if abs(gripper.y) < 0.01:
		gripper.y = 0
	if abs(gripper.z) < 0.01:
		gripper.z = 0
	
	print "\nFK"
	print "ServoAngles ", servoVal.yaw, servoVal.pitch, servoVal.elbow
	print "Spherical ", r, theta, thi
	print "Cartesian ", gripper.x, gripper.y, gripper.z
	
	return gripper
# end FK

def deg2pos(de):
	ret = de * 197
	ret = ret + 512
	return ret

def IK(position):
	x = position.x
	y = position.y
	z = position.z - z0
	r = math.sqrt(x**2 + y**2 + z**2)
	theta = math.atan2(x, y)
	thi = math.atan2(z, math.sqrt(x**2 + y**2))
	
	newServo = SERVO_POSITION()
	
	newServo.yaw = theta
	newServo.elbow = math.acos((2*(L1**2) - r**2)/(2*(L1**2)))
	newServo.pitch = thi - (math.pi/2 - (math.pi - newServo.elbow)/2)

	if abs(newServo.yaw) < 0.01:
		newServo.yaw = 0
	if abs(newServo.pitch) < 0.01:
		newServo.pitch = 0
	if abs(newServo.elbow) < 0.01:
		newServo.elbow = 0
	
	print "\nIK"
	print "Cartesian ", position.x, position.y, position.z
	print "Spherical ", r, theta, thi
	print "ServoAngles ", newServo.yaw, newServo.pitch, newServo.elbow
	
	return newServo	
# end IK

def translateR(position,direction):
	r = math.sqrt(position.x**2 + position.y**2)
	if direction == 1: # increase radius
		r = r + 5
	else:		  # decrease radius
		r = r - 5
	position.x = math.cos(math.atan2(position.y,position.x))
	position.y = math.sin(math.atan2(position.y,position.x))
	
	return position
	
# calcError - takes in current position and desired position returns error in radians
def calcError(desired, current):
  pixelError = -(desired - current)
  fov_rad = (fov*np.pi)/180.0
  radPerPixX = fov_rad/imgX
  return pixelError

def imageFunction(function):
	errorX = 20
	global servo
	x = 0
	y = x
	while abs(errorX) > 5:
		ret, frame = cap.read()
		resized = cv2.resize(frame, (imgX,imgY), interpolation = cv2.INTER_AREA)
		img = cv2.cvtColor(resized,cv2.COLOR_BGR2HSV)
		Rmask1 = cv2.inRange(img, red_lowerLim, red_upperLim)
		Rmask2 = cv2.inRange(img, np.array((175.,50.,50.)), np.array((180.,255.,255.)))
		Rmask = cv2.bitwise_or(Rmask1,Rmask2)
		#mask = cv2.inRange(img, lowerLim, upperLim)
		erosion = cv2.erode(Rmask,er,iterations = 1)
		dilation = cv2.dilate(erosion,di,iterations = 1)
		result = cv2.bitwise_and(resized, resized, mask = Rmask)
    		
      		cv2.circle(resized, (x,y), 2, (0,255,255),-1) # small yellow circle marks center of detected object
    		cv2.circle(resized, (cx + 40,cy), 2, (0,255,0),-1) # small green circle marks center of screen
		cv2.imshow("Gripper View", resized)
		cv2.waitKey(1)

		m = cv2.moments(dilation)
		
		if(m['m00'] != 0.0): # moment is detected
			if function == DETECT:
				GPIO.output(26, True)
				return 1
			x = int(m['m10']/m['m00'])
			y = int(m['m01']/m['m00'])
			errorX = calcError(cx + 40, x)
			errorY = calcError(cy, y)
			print errorX,errorY
			GPIO.output(26, False)
			if errorX > 0:
				servo.yaw = servo.yaw - 0.005
			else:
				servo.yaw = servo.yaw + 0.005
			position = FK(servo)
			pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
			ser1.write(pos)
			if errorY > 0:
				position = translateR(position, 1)
			else:
				position = translateR(position, 0)
				
			#servo = IK(position)

		else:
			if function == DETECT:
				GPIO.output(26, False)
				return 0
			print "!"
			GPIO.output(26, False)



#while IK(0.0835,-0.0623,0.1871):
#	time.sleep(0.01)

# Set desired joint values
#servo.yaw = -1.5708
#servo.pitch = 0.5
#servo.elbow = 2
#current = FK(servo)
#print current.x, current.y, current.z
#IK(current.x, current.y, current.z)
#pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(servo.elbow),520)
#ser1.write(pos)
#print ser1.read(4)

# Start Position
servo.yaw = -math.pi/2
servo.pitch = 0
servo.elbow = 1.4
servo.gripper = 350
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)
count = 0
while count < 90:
	if imageFunction(DETECT):
		count = count + 1
	else:
		count = count - 1
	if count < 0:
		count = 0
	time.sleep(0.1)

imageFunction(CENTER)

time.sleep(1)

servo.pitch = 0.6
servo.elbow = 1.4
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(1)
	
servo.gripper = 530
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(1)

servo.pitch = -0.3
servo.elbow = 1.6
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(1.2)

servo.yaw = 0
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(1.4)

servo.yaw = math.pi/4
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(1.3)

servo.yaw = math.pi/2
servo.pitch = 0
servo.elbow = 1.55
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(1.2)

servo.yaw = math.pi/2
servo.pitch = 0.4
servo.elbow = 1.8
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(0.5)

servo.gripper = 400
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)

time.sleep(1)

servo.yaw = math.pi/2
servo.pitch = 0
servo.elbow = 1.5
pos = struct.pack('hhhh',deg2pos(servo.yaw), deg2pos(-servo.pitch), deg2pos(-servo.elbow),servo.gripper)
ser1.write(pos)






import numpy as np
import cv2
import socket
import time

cap = cv2.VideoCapture(0)
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
host ="10.10.10.30"
port = 5005
buf =1024
addr = (host,port)

while(True):
  ret, frame = cap.read()
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  #resized = cv2.resize(gray, (160,120), interpolation = cv2.INTER_AREA)
  resized = cv2.resize(gray, (640,480), interpolation = cv2.INTER_AREA)
  cv2.imwrite("roboImage.jpg", resized)
  #cv2.imshow('frame',gray)
  f=open ("roboImage.jpg", "rb") 
  data = f.read(buf)
  while (data):
    if(sock.sendto(data,addr)):
      data = f.read(buf)
  f.close()
  sock.sendto("end image",addr)
  #time.sleep(.05)
  
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

cap.release()
cv2.destroyAllWindows()

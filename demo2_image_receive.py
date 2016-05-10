import socket
import sys
import Image
import time
import cv2

host="10.10.10.30"	#address to lissen computer
port = 5005
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((host,port))

addr = (host,port)
buf=1024

count = 0
cv2.namedWindow("RoboView", cv2.WINDOW_AUTOSIZE)

while True:
	bytes = 0
	f = open("ImgReceive.jpg","wb")
	#startstamp = time.time()
	data,addr = sock.recvfrom(buf)
	while(data):
		f.write(data)
		sock.settimeout(6)
		#endstamp = time.time()
		bytes += len(data)
		data,addr = sock.recvfrom(buf)
		if data == "end image":
			f.close()
			image = cv2.imread("ImgReceive.jpg", cv2.CV_LOAD_IMAGE_UNCHANGED)
			try:
				cv2.imshow("RoboView", image)
			except:
				e = sys.exc_info()[0]
			cv2.waitKey(20)
			break





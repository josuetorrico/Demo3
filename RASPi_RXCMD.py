##################
#ECE599 - Demo2 (ARM)
#David Gregory
##################
# Receiver Code - Run on Raspberry Pi#


import socket
import serial
#import time
#from timeit import default_timer as timer

#UDP_IP = '127.0.0.1' #For Testing
UDP_IP = "10.10.10.15"  #Address of PI(address to listen for), commands come from User
UDP_PORT = 5010
#CMD_PAK = {'RW':0, 'LW':0} #packet

#Open up a socket connection w/ PC
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

sock.bind((UDP_IP, UDP_PORT))

print'Waiting for Command'

# Listen for data on assigned port
while True:

    #RECEIVE command from remote control
	rxdata, address = sock.recvfrom(1024) # buffer size is 1024 bytes
	
	print(rxdata)
	
	######################################
	###  SEND Command to Open_CM over USB ###
	# For Raspberry Pi
	ser1 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
	
	if ser1.isOpen():
		ser1.write(rxdata)  #Send data packet
		response = ser1.read(ser1.inWaiting())
	
	print(response) 	#Echo should be returned from OpenCM
	
	ser1.close()
	########################################
	


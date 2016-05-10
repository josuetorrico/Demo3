##################
#ECE599 - Arm Demo
#David Gregory
##################
# Transmitter Code - Run on VM#
# Used to send single command to Pi # 

import socket
import sys
import tty, termios

UDP_IP = "10.10.10.15" #Target IP (destination); RPi
UDP_PORT = 5010

# Setup socket connection to Raspberry Pi
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

# Setup getch()

def getch():
        """getch() -> key character

        Read a single keypress from stdin and return the resulting character. 
        Nothing is echoed to the console. This call will block if a keypress 
        is not already available, but will not wait for Enter to be pressed. 

        If the pressed key was a modifier key, nothing will be detected; if
        it were a special function key, it may return the first character of
        of an escape sequence, leaving additional characters in the buffer.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
	    #print ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch




while True:
  #CMD = raw_input("Type CMD:")
  CMD = getch()
  
  #TX
  print CMD
  sock.sendto(CMD, (UDP_IP, UDP_PORT))









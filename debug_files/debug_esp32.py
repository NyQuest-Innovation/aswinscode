import struct
import binascii
import serial
import time
from datetime import datetime
ser=serial.Serial("COM7", 115200, timeout=10) #Establish connection with the PORT of HOST computer
    
now = datetime.now()
file_name = "debug-A12XA2000136-"+now.strftime("%d%m%Y")+".txt"
prev_date = now.strftime("%d")

while True:
	data = ser.readline() 
	time = datetime.now()
	if prev_date!=time.strftime("%d"):
	    file.close()
	    prev_date=time.strftime("%d")
	    file_name = "debug-12-"+time.strftime("%d%m%Y")+".txt"
	file=open(file_name,"a")
	print(time,data) 
	file.write("\n")
	file.writelines(str(time)+" "+str(data))
	file.close()
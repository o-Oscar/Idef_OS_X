import serial
import struct
#import matplotlib.pyplot as plt
import numpy as np
import time
"""
s = struct.pack('f', 1.)        # read up to ten bytes (timeout)
for i in range(4):
	print(bin(s[i]))

"""

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=3)

skipped_nb = 0
dumped_nb = 0

v_rot = np.asarray([0, 0, 0])
up_vect = np.asarray([0, 0, 1])

def read_float():
	s = ser.read(4)
	return struct.unpack('f', s)[0]

def read_imu ():
	global skipped_nb
	global dumped_nb
	
	waiting_byte = ser.inWaiting()
	skipped_nb = waiting_byte - waiting_byte%(4*6+1)
	ser.read(skipped_nb)
	
	#ser.flushInput()
	x = ser.read()
	dumped_nb = 0
	while not x == b'f':
		x = ser.read()
		dumped_nb += 1
	
	v_rot = np.asarray([read_float() for i in range(3)])
	up_vect = np.asarray([read_float() for i in range(3)])
		
	
	


if __name__ == "__main__":
	while (True):
		read_imu ()
		print(up_vect)
	
	"""
	with serial.Serial('/dev/ttyUSB0', 9600, timeout=3) as ser:
		
		all_data = []
		
		for i in range(10000):
			waiting_byte = ser.inWaiting()
			to_flush = waiting_byte - waiting_byte%(4*6+1)
			ser.read(to_flush)
			#ser.flushInput()
			print(ser.inWaiting())
			x = ser.read()
			while not x == b'f':
				x = ser.read()
				print("dump")
			
			data = []
			for i in range(6):
				s = ser.read(4)
				data.append(struct.unpack('f', s)[0])
			
			all_data.append(data)

			#print(np.asarray(all_data[-1]).reshape((3,3)))
			print(np.asarray(all_data[-1]))
			print()
			time.sleep(0.3)
	"""		
	#plt.plot(all_data)
	#plt.show()

import serial
import struct
#import matplotlib.pyplot as plt
import numpy as np
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)

skipped_nb = 0
dumped_nb = 0

v_rot = np.asarray([0, 0, 0])
up_vect = np.asarray([0, 0, 1])

def read_float():
	s = ser.read(4)
	return struct.unpack('f', s)[0]

"""
def read_float():
	for i in range(8):
		s = ser.read()
	return 0
"""
all_up = []
all_v = []
all_t = []

def read_imu (debug=False):
	global skipped_nb
	global dumped_nb
	global up_vect
	global v_rot
	start = time.time()
	waiting_byte = ser.inWaiting()
	skipped_nb = waiting_byte - waiting_byte%(4*6+1)
	#ser.read(skipped_nb)
	n_read = 0
	while ser.inWaiting() > 4*6+1:
		n_read += 1
		x = ser.read()
		dumped_nb = 0
		while not x == b'f':
			x = ser.read()
			dumped_nb += 1
		#print(2, time.time()-start)
		
		new_v_rot = np.asarray([read_float() for i in range(3)])
		#v_rot_stupid = np.asarray([read_float() for i in range(3)])
		
		raw_up_vect = [read_float() for i in range(3)]
		new_up_vect = np.asarray([-raw_up_vect[0], -raw_up_vect[1], -raw_up_vect[2]])
		
		lamb_up = 0.1/(1.3)**5
		lamb_v = lamb_up
		up_vect = up_vect*(1-lamb_up) + new_up_vect*lamb_up
		v_rot = v_rot*(1-lamb_v) + new_v_rot*lamb_v
	#up_vect = np.asarray([0, 0, 1])
	print(n_read)
	#print(3, time.time()-start)
	
	if debug:
		all_up.append(up_vect[:])
		all_v.append(v_rot[:])
		all_t.append(time.time())

if __name__ == "__main__":
	for i in range(30*15):
		read_imu (debug=True)
	
	all_up = np.asarray(all_up)
	all_v = np.asarray(all_v)
	all_t = np.asarray(all_t)
	print(all_up.shape)
	print(all_v.shape)
	print(all_t.shape)
	np.save("data/all_up.npy", all_up)
	np.save("data/all_v.npy", all_v)
	np.save("data/all_t.npy", all_t)
	
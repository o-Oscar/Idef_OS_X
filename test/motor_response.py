"""
ssh -L 8765:127.0.0.1:8765 pi@10.221.112.37

sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/test
python3 motor_response.py
"""

import numpy as np
import time

import backend as b

def test_kd_gain ():
	MOTOR_ID = 3

	b.init_bus()

	b.set_pid (MOTOR_ID, 6, 0, 50, 0, 50, 50)

	p0 = b.actuator_pos(MOTOR_ID)
	p1 = p0 + np.pi/10

	T = 2

	# startup
	b.position_control(MOTOR_ID, p0)
	print("coucou")
	time.sleep(2)

	# actual test
	to_save = [[time.time(), b.actuator_pos(MOTOR_ID)]]
	print("starting test")
	start = time.time()
	b.position_control(MOTOR_ID, p1)
	while time.time() < start + T:
		to_save.append([time.time(), b.actuator_pos(MOTOR_ID)])
	to_save = np.asarray(to_save)
	np.save("data.npy", to_save)



	# end
	b.position_control(MOTOR_ID, p0)
	print("coucou")
	time.sleep(2)

	b.turn_off(MOTOR_ID)

def test_kp_gain ():
	MOTOR_ID = 3

	b.init_bus()

	b.set_pid (MOTOR_ID, 6, 0, 50, 0, 50, 50)

	p0 = b.actuator_pos(MOTOR_ID)

	T = 10

	# startup
	b.position_control(MOTOR_ID, p0)
	print("starting countdown...")
	time.sleep(T)
	print(b.actuator_pos(MOTOR_ID)-p0)

	b.turn_off(MOTOR_ID)
	
	"""
	results : for a load at 20cm with a reduction ratio of 24/30 and pid = (6, 0, 50, 0, 50, 50)
	1kg -> 0.03033, 0.02795, 0.03004 rad
	2kg -> 0.06600, 0.06367, 0.06623 rad
	3kg -> 0.10285, 0.10567, 0.10623 rad
		Fitted --> kp = 72.13190744693489 N.rad-1
	
	results : for a load at 20cm with a reduction ratio of 24/30 and pid = (6, 0, 25, 0, 50, 50)
	2kg -> 0.12956
	"""

def print_kp_results ():
	l = 0.2
	g = 9.81
	m = np.asarray([1, 1, 1, 2, 2, 2, 3, 3, 3])
	r = 24/30
	torque = l*m*g/r
	delta_pos = np.asarray([0.03033, 0.02795, 0.03004, 0.06600, 0.06367, 0.06623, 0.10285, 0.10567, 0.10623])
	
	# torque = kp*delta_pos
	kp = np.sum(delta_pos * torque) / np.sum(delta_pos * delta_pos)
	
	print("kp = {} N.m.rad-1".format(kp))
	
	l = 0.4
	m = 3
	g = 9.81
	tau = l*g*m
	print("max torque motor 1, 2 : {} N.m".format(tau))
	l = 0.2
	m = 5
	g = 9.81
	tau = l*g*m
	print("max torque motor 3 : {} N.m".format(tau))

def lock_motor ():
	MOTOR_ID = 1

	b.init_bus()

	b.set_pid (MOTOR_ID, 6, 0, 50, 0, 50, 50)

	p0 = b.actuator_pos(MOTOR_ID)

	b.position_control(MOTOR_ID, p0)
	print("enter to end.")
	input()
	
	b.turn_off(MOTOR_ID)
	
	

# lock_motor();
	
# test_kp_gain()
print_kp_results()

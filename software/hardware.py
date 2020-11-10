
import kinematics
import time
import numpy as np

import init
import backend as b

motors_id = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]





motor_pose = None
def startup ():
	global motor_pose 
	motor_pose = init.load_motor_pose()
	
	b.init_bus()
	
	if not init.check_start (motors_id, motor_pose):
		raise NameError("Start position of motor {} is too far from saved rest position. Please re-set rest and zero pose.".format(str(motor_id)))
	
	
	for motor_id in motors_id:
		b.position_control (motor_pose["zero"][str(motor_id)], 0.5, motor_id)
	
		

def shutdown ():
	
	for motor_id in motors_id:
		b.position_control (motor_pose["rest"][str(motor_id)], 0.5, motor_id)
	
	input ("Enter to finish everything")
	
	for motor_id in motors_id:
		b.turn_off(motor_id)




def step ():
	pass




# Utility functions

def calc_motor_targ (start_pos, end_pos, dt, lamb=0.1):
    delta = end_pos-start_pos
    targ_pos = end_pos + delta*lamb
    targ_vel = delta/dt
    return targ_pos, abs(targ_vel)

import motors
import numpy as np
import imu
import kinematics

all_motor_pos = []
all_motor_targ = []
kin = kinematics.Kinematics()
standard_rot = kin.standard_rot(np.asarray([1] * 12))

def record (act, phases, are_motors_on):
	motor_pos = motors.get_pos(fetch=True) if are_motors_on else [0 for i in range(12)]
	motor_pos = np.asarray(motor_pos)
	all_motor_pos.append(motor_pos * standard_rot)
	
	targ_pose = kin.motor_pos (act, phases)
	all_motor_targ.append(targ_pose * standard_rot)

def save ():
	global all_motor_pos
	global all_motor_targ
	
	all_motor_pos = np.asarray(all_motor_pos)
	all_motor_targ = np.asarray(all_motor_targ)
	np.save("data/records/motors/pos.npy", all_motor_pos)
	np.save("data/records/motors/targ.npy", all_motor_targ)
	
	
	imu.all_up = np.asarray(imu.all_up)
	imu.all_v = np.asarray(imu.all_v)
	imu.all_t = np.asarray(imu.all_t)
	
	np.save("data/records/imu/all_up.npy", imu.all_up)
	np.save("data/records/imu/all_v.npy", imu.all_v)
	np.save("data/records/imu/all_t.npy", imu.all_t)
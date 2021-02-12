import motors
import imu
import numpy as np
import kinematics

""""
obs : 

Joint targpos
Joint pos
Foot Phase
Local up
rot speed

POS_VEL_CMD,
ROT_VEL_CMD,
"""

last_target_joint_pos = None
lamb = 1 # 0.403

def get_obs (pos_vel_cmd, rot_vel_cmd, target_joint_pos, joint_pos, phases):
	#joint_pos = np.asarray(kinematics.standard_rot(motors.get_pos(fetch=True)))
	global last_target_joint_pos
	if last_target_joint_pos is None:
		last_target_joint_pos = target_joint_pos
	else:
		last_target_joint_pos += lamb * (target_joint_pos - last_target_joint_pos)
		
	up_vect = np.asarray(imu.up_vect)
	rot_speed = np.asarray(imu.v_rot)
	pos_vel_cmd = np.asarray(pos_vel_cmd)
	rot_vel_cmd = np.asarray(rot_vel_cmd)
	sin_phase = np.sin(phases)
	cos_phases = np.cos(phases)
	
	#obs = (last_target_joint_pos, joint_pos, sin_phase, cos_phases, up_vect, rot_speed, pos_vel_cmd, rot_vel_cmd)
	obs = (last_target_joint_pos, joint_pos-last_target_joint_pos, sin_phase, cos_phases, up_vect, pos_vel_cmd, rot_vel_cmd)
	#obs = (last_joint_pos, sin_phase, cos_phases, up_vect, pos_vel_cmd, rot_vel_cmd)
	obs = np.concatenate(obs)
	obs = obs.reshape((1, 1, -1))
	
	return obs

def update_readings ():
	#motors.get_pos()
	imu.read_imu(debug=True)
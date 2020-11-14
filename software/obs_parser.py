import motors
import imu

#motors.get_speeds()

""""
obs : 

Joint pos
Joint vel
Local up
rot speed

POS_VEL_CMD,
ROT_VEL_CMD,
"""

class Env:
	def __init__ (self):
		self.act_dim = 12
		self.obs_dim = 33

def get_obs (pos_vel_cmd, rot_vel_cmd):
	joint_pos = np.asarray(motos.get_pos())
	joint_speed = np.asarray(motors.get_speeds())
	up_vect = np.asarray(imu.up_vect)
	rot_speed = np.asarray(imu.v_rot)
	pos_vel_cmd = np.asarray(pos_vel_cmd)
	rot_vel_cmd = np.asarray(rot_vel_cmd)
	
	obs = (joint_pos, joint_speed, up_vect, rot_speed, pos_vel_cmd, rot_vel_cmd)
	obs = np.concatenate(obs)
	obs = obs.reshape((1, 1, -1))
	
	return obs
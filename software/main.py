"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
source activate psc
py main.py

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0
"""

import time
import numpy as np
import os

from actor import SimpleActor, MixtureOfExpert, LSTMActor
import obs_parser
import motors
import kinematics

#up_pose = kinematics.motor_pos ([0.5, 0.5, 0.5] * 4)
#down_pose = kinematics.motor_pos ([0.5, 0.4, 0.5] * 4)
#trans_speed = np.abs((up_pose-down_pose)/dt)

def action_at_speed (act, dt=1, lamb=0):
	cur_pose = np.asarray(motors.get_pos())
	targ_pose = kinematics.motor_pos (act) * 1
	delta_pose = targ_pose-cur_pose
	targ_pose += delta_pose*lamb
	
	speed = np.minimum(np.abs(delta_pose/dt), 5)
	motors.goto (targ_pose, targ_vel=speed)

if __name__ == "__main__":
	
	# actor
	"""
	env = obs_parser.Env()
	actor_type = "simple"
	path = os.getcwd() + "/models/expert_1ms/{}"
	
	if actor_type=="mix":
		primitives = [SimpleActor(env) for i in range(2)]
		actor = MixtureOfExpert(env, primitives, debug=True)
	elif actor_type == "simple":
		actor = SimpleActor(env)
	elif actor_type == "lstm":
		actor = LSTMActor(env)
	
	actor.load(path)
	#actor.save(path)
	"""
	
	
	motors.check_configuration ()
	
	input("Enter to start the dog")
	action_at_speed([0.5, 0.5, 0.3] * 4)
		
	input("Enter to go to rest")
	print(kinematics.standard_rot(motors.get_pos()))
	motors.goto_rest ()
	input("Enter to stop the dog")
	
	motors.disengage ()
	
	
	
	
	obs_parser.update_readings ()
	obs = obs_parser.get_obs([1, 0], [0])
	print(obs)
	print(actor.model(actor.scaler.scale_obs(obs)))
	
	
	
	

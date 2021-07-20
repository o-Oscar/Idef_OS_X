"""

sudo apt-get install rsync
rsync -auvn --exclude logs/ ~/workspace/Idef_OS_X/software/ pi@192.168.1.18:~/workspace/Idef_OS_X/software/


ssh pi@192.168.1.18 ls workspace/Idef_OS_X/software/src/logs/
scp pi@192.168.1.18:~/workspace/Idef_OS_X/software/src/logs/2021_07_18_16h10m07s_LogFile.hdf5 ~/workspace/Idef_OS_X/software/src/logs/
scp pi@192.168.1.18:~/workspace/Idef_OS_X/software/src/logs/2021_07_18_16h10m07s_LogFile.hdf5 ~/workspace/rl_toolbox/src/logs/

ssh -L 8765:127.0.0.1:8765 pi@192.168.1.18

ssh -L 8765:127.0.0.1:8765 pi@192.168.1.40      maison oscar
ssh -L 8765:127.0.0.1:8765 pi@192.168.43.59     telephone oscar
ssh -L 8765:127.0.0.1:8765 pi@10.221.112.37     boyd
ssh -L 8765:127.0.0.1:8765 pi@192.168.1.18		rue descartes
mdp : raspberry


sudo ip link set can0 up type can bitrate 1000000
cd ~/workspace/Idef_OS_X/software
python3 main.py

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0

"""
import sys

if len(sys.argv) <= 1 and False:
	print()
	print("Usage: python3 main.py <option>")
	print()
	print("    gotozero : wip")
	print("    neuralnet : wip")
	print()
	exit()

from idefX import IdefX

import time
import numpy as np

import torch as th
import student_models

import os

xs = np.asarray([1, 0, 0] * 4)
ys = np.asarray([0, 1, 0] * 4)
zs = np.asarray([0, 0, 1] * 4)

if __name__ == "__main__":
	
	idefX = IdefX(enable_motors=True)
	model = student_models.conv_student_model(idefX.obs_gen.obs_dim)
	model.load_state_dict(th.load(os.path.join("src", "models", "exp_0", "model")))

	action = np.asarray([0]*12) # + xs * 0.3

	print("Enter to start IdefX")
	input()
	idefX.start (action)

	print("Enter to test IdefX")
	input()

	# idefX.update_state(action=action, update_phase=False)
	# obs = idefX.get_obs()
	all_obs = []


	last_loop = time.time()
	found_error = False
	while idefX.state.frame < 30*10 and not found_error:
		idefX.state.target_speed = np.asarray([0.4, 0, 0])
		idefX.state.target_rot_speed = np.asarray([0, 0, 0])

		idefX.update_state(action=action)
		
		obs = idefX.get_obs()
		# print("obs_shape :", obs.shape)
		all_obs.append(obs)
		stack_len = min(len(all_obs), 100)
		obs_stack = np.expand_dims(np.stack(all_obs[-stack_len:], axis=0), axis=0)
		# print("obs_stack_shape :", obs_stack.shape)
		
		with th.no_grad():
			action = model(th.tensor(obs_stack.astype(np.float32))).numpy()[0,-1]
		# action = action
		# print("action_shape :", action.shape)

		if np.isnan(np.sum(action)):
			print("found Nan in array. exiting.")
			found_error = True
			break
			

		if idefX.motors_enabled:
			idefX.exec_action(action, idefX.state.phase)

		
		while time.time()-last_loop < 1/30:
			pass
		last_loop = time.time()


	print("Enter to go to stop")
	input()

	print("Stopping IdefX")
	idefX.stop ()

	print("Going to rest pose")
	idefX.go_to_rest ()

	print("Shutting down")
	idefX.shutdown()

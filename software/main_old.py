"""
ssh -L 8765:127.0.0.1:8765 pi@192.168.1.18
raspberry

sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
python3 main.py

arp -a
ssh -L 8765:127.0.0.1:8765 pi@192.168.1.40      maison oscar
ssh -L 8765:127.0.0.1:8765 pi@192.168.43.59     telephone oscar
ssh -L 8765:127.0.0.1:8765 pi@10.221.112.37     boyd
ssh -L 8765:127.0.0.1:8765 pi@192.168.1.18		rue descartes
ping 192.168.1.40
mdp : raspberry


sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0

cd C://Users/oscbo/Documents/Travail/PSC/idefX/v3/Idef_OS_X/software
conda activate psc
python client_controller.py
"""

import time
import numpy as np
import os

#from actor import SimpleActor, MixtureOfExpert, LSTMActor
#import tensorflow.lite as tflite
import obs_parser
import motors
import kinematics
import lite_model as actor
import server
import movement_planner
import recorder
import imu

model_path = "/models/baseline/{}" 

kin = kinematics.Kinematics()
recorder.standard_rot = kin.standard_rot([1]*12)

xs = np.asarray([1, 0, 0]*4)
ys = np.asarray([0, 1, 0]*4)
zs = np.asarray([0, 0, 1]*4)

# zero_act = np.asarray([0.5, 0.5, 0.4] * 4)
zero_act = np.asarray([0., 0., 0.] * 4)
act_offset = np.asarray([0] * 12)

# --- config ---
foot_f0 = np.asarray([1, 1, 1, 1]) * 1.

def exec_action (act, phase):
	targ_pose = kin.standard_rot(kin.calc_joint_target (act, phase))
	#targ_pose -= 0.01 * np.asarray([1, 0, 0, -1, 0, 0, -1, 0, 0, 1, 0, 0])
	
	motors.position_control (targ_pose)

def exec_timed_action (act, phase, dt=1, lamb=0, fetch=False):
	cur_pose = np.asarray(motors.get_pos(fetch=fetch))
	targ_pose = kin.standard_rot(kin.calc_joint_target (act, phase))
	delta_pose = targ_pose-cur_pose
	targ_pose += delta_pose*lamb
	if dt == 0:
		speed = delta_pose * 0 + 10
	else:
		speed = np.minimum(np.abs(delta_pose/dt), 10)
	
	#targ_pose -= 0.01 * np.asarray([1, 0, 0, -1, 0, 0, -1, 0, 0, 1, 0, 0])
	
	motors.position_control_at_speed (targ_pose, speed)

if __name__ == "__main__":
	server.start_server()
	
	frame = 0
	phase = 0
	speed_target = [0.4, 0]
	rot_target = 0
	obs_acc = []
	actual_act = zero_act
	last_loop = 0
	exercise_start_id = 0
	
	exercise_done = False
	exercise_started = True # False
	advance_foot_phases = True
	
	start_foot_phases = True
	multiple_planner_run = False
	
	neural_net_action = True
	planner_action = False
	
	
	# --- debug ---
	last_time = time.time()
	all_times = [[], [], [], []]
	move_motors = True
	
	
	path = os.getcwd() + model_path
	actor.load (path)

	if move_motors:
		motors.check_configuration ()
		
		input("Enter to put dog at its zero")
		motors.set_strong_pid()
		exec_timed_action(actual_act, phase, dt=1, fetch=True)
		time.sleep(1)
		motors.set_lite_pid()
		# motors.set_strong_pid()
		exec_action(actual_act, phase)
	
	input("Enter to start the dog")
	print("Main program has started")
	i = 0
	while not exercise_done and i < 30*30:
		i += 1
		
		# --- advancing foot phases
		if advance_foot_phases:
			#2*np.pi*self.state.foot_f0*self.timeStep*self.frameSkip
			frame += 1
			phase = frame*2*np.pi/30
		
		# --- updating the target cmd ---
		cmd = server.update_cmd()
		#print(cmd)
		if "button_0" in cmd and cmd["button_0"] == 1:
			exercise_start_id = i
			exercise_started = True
			advance_foot_phases = start_foot_phases 
		elif "button_1" in cmd and cmd["button_1"] == 1:
			exercise_done = True
		if "axis_0" in cmd and "axis_1" in cmd and "axis_2" in cmd:
			max_speed_x = 1 # 0.4
			max_speed_y = 1 # 0.2
			speed_target = [-cmd["axis_1"]*max_speed_x, cmd["axis_0"]*max_speed_y]
			rot_target = -cmd["axis_2"]*0.4
			if True:
				speed_target = [0.4, 0]
				rot_target = 0
		
		# --- gathering observations ---
		start = time.time()
		obs_parser.update_readings ()
		all_times[0].append(time.time()-start)
		joint_pos = kin.standard_rot(motors.get_pos(fetch=True)) if move_motors else kin.calc_joint_target (actual_act, phase)
		new_obs = obs_parser.get_obs(speed_target, [rot_target], kin.calc_joint_target (actual_act, phase), joint_pos, phase) # <- right line
		# new_obs = obs_parser.get_obs(speed_target, [rot_target], kin.calc_joint_target (actual_act, phase), kin.calc_joint_target (actual_act, phase), phase)
		while len(obs_acc) <= 3:
			obs_acc.append(new_obs)
		obs_acc = obs_acc[1:]
		obs = np.concatenate(obs_acc, axis=1)
		# print(new_obs.shape, obs.shape)
		
		# --- default action ---
		actual_act = zero_act
		
		# --- fetching actor action ---
		if neural_net_action:
			if len(obs_acc) == 3:
				if exercise_started:
					start = time.time()
					# print(obs)
					act = actor.step(obs)
					# print(act)
					# print(act[-1])
					act = act[-1] * 1.
					act = np.maximum(np.minimum(act, 1), -1)
					# print(act)
					# print(act)
					#act = (act+1)/2
					#act = actor.step_old(obs)
					all_times[1].append(time.time()-start)
					
					neural_act = act.flatten()
				else:
					neural_act = zero_act
			else:
				neural_act = zero_act
			
			actual_act = neural_act
		
		# --- fetching other action ---
		if planner_action and exercise_started:
			actual_act, is_movement_done = movement_planner.get_action (i-exercise_start_id)
			exercise_started = not is_movement_done
			if is_movement_done and not multiple_planner_run:
				exercise_done = True
		
		# --- executing action ---
		start = time.time()
		if move_motors:
			exec_action(actual_act, phase)
		all_times[2].append(time.time()-start)
		
		# --- recording stuff ---
		recorder.record (actual_act, phase, move_motors)
		
		
		# --- logging ---
		all_times[3].append(time.time()-last_time)
		last_time = time.time()
		
		# --- waiting the right amount of time ---
		while time.time()-last_loop < 33/1000: # 1/30:
			pass
		last_loop = time.time()
	
	# --- return to zero ---
	end_phase = 0
	if move_motors:
		#input("return to zero")
		exec_action(zero_act, end_phase)
	
	# --- debug logging ---
	if len(all_times[0]) > 0:
		print()
		print("mean obs compute time :", np.mean(all_times[0]))
		print("median obs compute time :", np.median(all_times[0]))
		print("mean network compute time :", np.mean(all_times[1]))
		print("mean execution time :", np.mean(all_times[2]))
		print("median loop time :", np.median(all_times[3]))
		
	# --- save recorder data ---
	recorder.save ()
	
	if move_motors:
		input("Enter to put the dog to rest")
		end_phase = 0
		
		motors.set_strong_pid()
		exec_timed_action(zero_act, end_phase, dt=1, fetch=True)
		time.sleep(1)
		
		motors.goto_rest ()
		time.sleep(2)
		input("Enter to stop the dog")
		
		motors.disengage ()
	
	
	server.stop_server()
	time.sleep(1)
	

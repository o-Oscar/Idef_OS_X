"""
ssh -L 8765:127.0.0.1:8765 pi@10.221.112.37
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
python3 main.py

arp -a
ssh -L 8765:127.0.0.1:8765 pi@192.168.1.40      maison
ssh -L 8765:127.0.0.1:8765 pi@192.168.43.59     telephone
ssh -L 8765:127.0.0.1:8765 pi@10.221.112.37     boyd
ping 192.168.1.40
mdp : raspberry

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0
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

kin = kinematics.Kinematics()
recorder.standard_rot = kin.standard_rot([1]*12)

zero_act = np.asarray([0.5, 0.5, 0.2] * 4)
act_offset = np.asarray([0] * 12)

# --- config ---
foot_f0 = np.asarray([1, 1, 1, 1]) * 1.5


def exec_action (act, phases):
	targ_pose = kin.motor_pos (act, phases)
	#targ_pose -= 0.01 * np.asarray([1, 0, 0, -1, 0, 0, -1, 0, 0, 1, 0, 0])
	
	motors.position_control (targ_pose)

def exec_timed_action (act, phases, dt=1, lamb=0, fetch=False):
	cur_pose = np.asarray(motors.get_pos(fetch=fetch))
	targ_pose = kin.motor_pos (act, phases)
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
	
	foot_phases = np.asarray([0, np.pi, np.pi, 0])
	speed_target = [0, 0]
	rot_target = 0
	obs_acc = []
	actual_act = zero_act
	last_loop = 0
	exercise_start_id = 0
	
	exercise_done = False
	exercise_started = False
	advance_foot_phases = False
	
	start_foot_phases = True
	multiple_planner_run = False
	
	neural_net_action = True
	planner_action = False
	
	
	# --- debug ---
	last_time = time.time()
	all_times = [[], [], [], []]
	move_motors = True
	
	
	path = os.getcwd() + "/models/exp_0/{}" 
	actor.load (path)

	if move_motors:
		motors.check_configuration ()
		
		input("Enter to put dog at its zero")
		motors.set_strong_pid()
		exec_timed_action(actual_act, foot_phases, dt=1, fetch=True)
		time.sleep(1)
		motors.set_lite_pid()
		exec_action(actual_act, foot_phases)
	
	input("Enter to start the dog")
	print("Main program has started")
	i = 0
	while not exercise_done: #  and i < 10:
		i += 1
		
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
			max_speed_x = 0.4
			max_speed_y = 0.2
			speed_target = [-cmd["axis_1"]*max_speed_x, cmd["axis_0"]*max_speed_y]
			rot_target = -cmd["axis_2"]*0.5
		
		# --- gathering observations ---
		start = time.time()
		obs_parser.update_readings ()
		all_times[0].append(time.time()-start)
		joint_pos = motors.get_pos(fetch=True) if move_motors else [0]*12
		new_obs = obs_parser.get_obs(speed_target, [rot_target], kin.standard_rot(kin.motor_pos (actual_act, foot_phases)), kin.standard_rot(joint_pos), foot_phases)
		while len(obs_acc) <= 3:
			obs_acc.append(new_obs)
		obs_acc = obs_acc[1:]
		obs = np.concatenate(obs_acc, axis=2)
		#print(new_obs.shape, obs.shape)
		
		# --- fetching actor action ---
		if neural_net_action:
			if len(obs_acc) == 3:
				if exercise_started:
					start = time.time()
					act = actor.step(obs)
					#act = actor.step_old(obs)
					#print(act)
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
			exec_action(actual_act, foot_phases)
		all_times[2].append(time.time()-start)
		
		# --- recording stuff ---
		recorder.record (actual_act, foot_phases, move_motors)
		
		
		# --- logging ---
		all_times[3].append(time.time()-last_time)
		last_time = time.time()
		
		# --- advancing foot phases
		if advance_foot_phases:
			foot_phases += 2*np.pi*foot_f0/30
		
		# --- waiting the right amount of time ---
		while time.time()-last_loop < 1/30:
			pass
		last_loop = time.time()
	
	# --- return to zero ---
	end_phases = np.asarray([0, 0, 0, 0])
	if move_motors:
		#input("return to zero")
		exec_action(zero_act, end_phases)
	
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
		end_phases = np.asarray([0, 0, 0, 0])
		
		motors.set_strong_pid()
		exec_timed_action(zero_act, end_phases, dt=1, fetch=True)
		time.sleep(1)
		
		motors.goto_rest ()
		input("Enter to stop the dog")
		
		motors.disengage ()
	
	"""
	"""
	server.stop_server()
	time.sleep(1)
	

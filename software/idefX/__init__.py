import idefX.motors as motors
from idefX.kinematics import Kinematics
from idefX.recorder import Recorder
from idefX.imu import IMU
from idefX.state import State
from idefX.obs_gen import BestRealisticObsGenerator

import time
import numpy as np

class IdefX:

	
	def __init__ (self, enable_motors=False):
		self.motors_enabled = enable_motors
		self.is_started = False

		self.state = State ()
		self.kin = Kinematics()
		self.recorder = Recorder()
		self.imu = IMU()
		self.obs_gen = BestRealisticObsGenerator(self.state)
		self.obs_gen.reset()
		self.q_to_w_ = np.diag(np.asarray([-1, -1, 1, 1, 1, -1, 1, -1, 1, -1, 1, -1]))

		if self.motors_enabled:
			motors.check_configuration ()

		self.reset()


	def reset (self):
		self.state.frame = 0
		self.state.phase = 0
	

	def start (self, action):
		self.is_started = True
		self.recorder.open()
		
		if self.motors_enabled:
			motors.set_strong_pid()
			self.exec_timed_action(action, self.state.phase, dt=1, fetch=True)
			time.sleep(1)
			motors.set_lite_pid()
			# motors.set_strong_pid()
			self.exec_action(action, self.state.phase)


	def update_state (self, action, update_phase=True):
		self.imu.update()
		
		if self.motors_enabled:
			self.state.joint_rot = self.q_to_s(motors.get_pos(fetch=True))
		else:
			self.state.joint_rot = np.zeros((12,))
			
		self.state.joint_target = self.kin.calc_joint_target (action, self.state.phase)
		
		self.state.loc_rot_speed = self.imu.v_rot
		self.state.loc_up_vect = self.imu.up_vect
	
		self.record(action=action)
		# self.state.target_speed = np.asarray([1, 0, 0])*1
		# self.state.target_rot_speed = np.asarray([0, 0, 0])

		if update_phase:
			self.state.frame += 1
			self.state.phase += 2*np.pi*self.state.f0/30 # <- TODO : make that more flexible and less error-prone

	def get_obs (self):
		return self.obs_gen.generate()

	def record (self, **kwargs):
		self.recorder.add(time=time.time())
		self.recorder.add(up_vect=self.state.loc_up_vect, rot_vel=self.state.loc_rot_speed)
		self.recorder.add(joint_rot=self.state.joint_rot)
		self.recorder.add(**kwargs)

		self.recorder.commit()

	def stop (self):
		if self.motors_enabled:
			stop_action = np.zeros((12,))
			stop_phase = 0
			
			motors.set_lite_pid()
			self.exec_action(stop_action, stop_phase)
			# self.exec_timed_action(stop_action, stop_phase, dt=1, fetch=True)
			time.sleep(1)
		
		self.is_started = False
		self.recorder.close()
		self.reset()


	def go_to_rest (self):
		if self.motors_enabled:
			motors.set_strong_pid()
			motors.goto_rest ()
			time.sleep(2)
		self.is_started = False

	def shutdown (self):
		if self.motors_enabled:
			motors.disengage ()
		self.is_started = False
	





	def q_to_w (self, standard_q): # qpos, standard to world 
		return (self.q_to_w_ @ np.asarray(standard_q).reshape((-1, 1))).reshape(np.asarray(standard_q).shape)
	def q_to_s (self, standard_q): # qpos, world to standard 
		return (self.q_to_w_.T @ np.asarray(standard_q).reshape((-1, 1))).reshape(np.asarray(standard_q).shape)
	
	def exec_action (self, act, phase):
		if not self.motors_enabled:
			raise NameError("you are supposed to enable the motors before calling Idefx.exec_action")
		if not self.is_started:
			raise NameError("you have to start IdefX before executing a non timed action")

		targ_pose = self.q_to_w(self.kin.calc_joint_target (act, phase))
		motors.position_control (targ_pose)

	def exec_timed_action (self, act, phase, dt=1, lamb=0, fetch=True):
		if not self.motors_enabled:
			raise NameError("you are supposed to enable the motors before calling Idefx.exec_timed_action")

		cur_pose = np.asarray(motors.get_pos(fetch=fetch))
		targ_pose = self.q_to_w(self.kin.calc_joint_target (act, phase))
		delta_pose = targ_pose-cur_pose
		targ_pose += delta_pose*lamb
		if dt == 0:
			speed = delta_pose * 0 + 10
		else:
			speed = np.minimum(np.abs(delta_pose/dt), 10)
		
		motors.position_control_at_speed (targ_pose, speed)

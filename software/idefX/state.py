import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import os
from idefX.reference import ReferenceBag

class State:
	def __init__ (self):
		self.reset()
		self.reference_bag = ReferenceBag()
		self.sim_args = {}
		
	def reset (self, phase=0, frame=0):
		self.joint_rot = [0]*12
		self.joint_target = [0]*12
		# self.joint_rot_speed = [0]*12 # takes too long to retrieve
		# self.joint_torque = [0]*12 # takes too long to retrieve

		self.phase = phase
		self.frame = frame
		self.f0 = 1.
		# self.f0 = 1.5
		
		self.loc_rot_speed = [0, 0, 0]
		self.loc_up_vect = [0, 0, 1]
	
		self.target_speed = np.asarray([1, 0, 0])*1
		self.target_rot_speed = np.asarray([0, 0, 0])
		
		# self.friction_f = 0.3
import numpy as np
from idefX.reference import ReferenceBag

class Kinematics():
	def __init__(self):
		self.reference_bag = ReferenceBag()
		self.zero_pos = np.asarray([0, np.pi*0.2, -np.pi*0.4] * 4)
		self.full_range = np.asarray([np.pi*25/180, np.pi*45/180, np.pi*45/180] * 4)
		self.adjustement_range = np.asarray([np.pi*25/180, np.pi*45/180, np.pi*45/180] * 4)# / 2
		
		# self.use_reference = True
		self.ref_name = "in_place"
			
	def calc_joint_target (self, raw_action, phases, use_reference=True):
		if use_reference:
			qpos, qvel = self.reference_bag.get_ref(self.ref_name, phases)
			return qpos[-12:] + raw_action*self.adjustement_range
		else:
			return self.zero_pos + raw_action*self.full_range


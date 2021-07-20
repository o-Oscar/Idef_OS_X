import os
import numpy as np
import h5py
from datetime import datetime

class Recorder:
	def __init__ (self):
		self.dset_sizes = {
				"time"		: 1,
				"action"	: 12,
				"up_vect"	: 3,
				"rot_vel"	: 3,
				"joint_rot"		: 12,
			}
		
		self.added_data = {}
	
	def open (self):
		now = datetime.now()
		dt_string = now.strftime("%Y_%m_%d_%Hh%Mm%Ss")
		self.filename = "{}_LogFile.hdf5".format(dt_string)
		self.path = os.path.join("src", "logs", self.filename)

		self.f = h5py.File(self.path, "w")
		# self.dset = self.f.create_dataet("my_logs", )

		self.dsets = {key:self.f.create_dataset(key, (0, size), maxshape=(None, size)) for key, size in self.dset_sizes.items()}
		self.dset_n_elems = {key:0 for key, size in self.dset_sizes.items()}

	def add (self, **kwargs):
		for key, value in kwargs.items():
			self.added_data[key] = value
	
	def commit (self):
		self.record(self.added_data)
		self.added_data = {}

	def record (self, data):
		for key in data.keys():
			if not key in self.dsets:
				print("WARNING : Trying to log unknown key \"{}\". Data is being lost.".format(key))

		for key, dset in self.dsets.items():
			if not key in data:
				print("WARNING : key \"{}\" was not found in data. The log might become unusable.".format(key))
			else:
				if self.dset_n_elems[key] >= len(dset):
					self.resize_dataset(dset)
				dset[self.dset_n_elems[key]] = np.asarray(data[key]).reshape((self.dset_sizes[key], ))
				self.dset_n_elems[key] += 1

	def resize_dataset (self, dset):
		dset.resize(len(dset) + 1, axis=0) # constant time -> less risk of big spikes. 
		# dset.resize(len(dset)*2, axis=0) # exponential size -> might take less time overall, but risk of big spike. 

	def close (self):
		self.f.close()
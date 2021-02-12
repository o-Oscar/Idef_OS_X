import numpy as np
import matplotlib.pyplot as plt



all_pos = np.load("records/motors/pos.npy")
plt.plot(all_pos)

all_targets = np.load("records/motors/targ.npy")
plt.plot(all_targets)

plt.show()













"""
all_up = np.load("data/all_up.npy")
all_v = np.load("data/all_v.npy")

for i, name in enumerate("xyz"):
	plt.plot(all_up[:,i], label="up_"+name)

for i, name in enumerate("xyz"):
	plt.plot(all_v[:,i], label="v_"+name)
plt.legend()
plt.show()
"""
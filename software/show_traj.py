
import h5py
import os
import numpy as np
import matplotlib.pyplot as plt

filename = "coucou"#.format(dt_string)
path = os.path.join("src", "logs", filename)

f = h5py.File(path, "r")

for key, value in f.items():
	print(key)
	plt.plot(value)
	plt.show()
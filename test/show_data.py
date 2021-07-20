"""
conda activate psc
cd C:\\Users\\oscbo\\Documents\\Travail\\PSC\\idefX\\v2\\Idef_OS_X\\test
python show_data.py
"""
import matplotlib.pyplot as plt
import numpy as np

data = np.load("data.npy")
all_t = data[:,0]
all_t -= all_t[0]
all_r = data[:,1]
all_r = all_r[-1]-all_r
all_r /= all_r[0]
all_l = np.log(all_r)

i0 = 0
while all_t[i0] < 0.25:
	i0 += 1
tau = -all_t[i0]/all_l[i0]
print("decay period : {} s".format(tau))
# print(i0)


plt.plot(all_t, all_l)
# plt.plot(all_t, all_r)
plt.show()

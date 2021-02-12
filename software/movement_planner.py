
import numpy as np

#main_movement = np.load('data/movements/x_step.npy')
#main_movement = np.load('data/movements/x_falling.npy')
#main_movement = np.load('data/movements/true_step.npy')
#main_movement = np.load('data/movements/nothing_20.npy')

#main_movement = np.load('data/movements/pompe.npy')
#main_movement = np.load('data/movements/rot_z.npy')
#main_movement = np.load('data/movements/rot_y.npy')
main_movement = np.load('data/movements/rot_x.npy')

def get_action (i):
	if i < 0:
		return main_movement[0], True
	elif i >= len(main_movement):
		return main_movement[-1], True
	else:
		return main_movement[i], False

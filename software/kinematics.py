
from leg_ik import Leg
import numpy as np

all_legs = [Leg(inv_elbow=True),
			Leg(inv_elbow=False),
			Leg(inv_elbow=False),
			Leg(inv_elbow=True)]
			


def motor_pos (raw_action):
	legs_actions = split_legs(raw_action)
	to_return = []
	for leg, leg_action in zip(all_legs, legs_actions):
		to_return += leg.motor_pos (leg_action)
	return np.asarray(to_return)

def split_legs (action):
	to_return = []
	for i in range(4):
		to_return.append([action[i*3+j] for j in range(3)])
	return to_return

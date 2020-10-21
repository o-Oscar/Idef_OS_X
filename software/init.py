"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
source activate psc
python init.py rest
python init.py zero 1 2 3
"""
"""
import motors
import kinematics
import time
import numpy as np
"""
import sys
import json

def load_motor_pose ():
	with open("motor_pose.txt") as f:
		pose = json.loads(''.join(f.readlines()))
	#print(json.dumps({}))
	return pose

if __name__ == "__main__":
	if len(sys.argv) < 3:
		raise NameError("Not enough args")
		
	elif sys.argv[1] == "rest":
		if len(sys.argv) < 3:
			raise NameError("Need motors args to operate")
		else:
			motors_id = [int(x) for x in sys.argv[2:]]
			print(motors_id)
			motor_pose = load_motor_pose()
			for id in motors_id:
				motor_pose[id] = motors.get_pos
			
			
			
			
			
		
	elif sys.argv[1] == "zero":
		if len(sys.argv) < 3:
			raise NameError("Need motors args to operate")
		else:
			motors_id = [int(x) for x in sys.argv[2:]]
			print(motors_id)
	else:
		raise NameError("arg not recognized")
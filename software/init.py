"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
source activate psc
python init.py set

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0
"""

import backend as b
import sys
import json
import time

def load_motor_pose ():
	with open("motor_pose.txt") as f:
		pose = json.loads(''.join(f.readlines()))
	#print(json.dumps({}))
	return pose

def save_motor_pose (pose):
	with open("motor_pose.txt", "w") as f:
		f.write(json.dumps(pose))

def check_start (all_motor_id, motor_pose):
	for motor_id in all_motor_id:
		start_position = b.actuator_pos(motor_id))
		if abs(start_position - motor_pose["rest"][str(motor_id)]) > 3.14/6/10:
			return False
	return True

if __name__ == "__main__":
	if len(sys.argv) < 3:
		raise NameError("This programm needs at leat two arguments : (set/check) motor_id_0 motor_id_1 motor_id_2 ...")
	elif sys.argv[1] == "set":
		
		b.init_bus ()
		
		choosen_motors_id = [int(x) for x in sys.argv[2:]]
		motor_pose = load_motor_pose()
		
		# choosing the rest pose
		for motor_id in all_motor_id:
			b.set_zero_torque(motor_id)
		
		
		input("Put motors {} at their rest position.".format(all_motor_id))
		for motor_id in choosen_motors_id:
			motor_pose["rest"][str(motor_id)] = b.actuator_pos(motor_id)
		
		input("Put motors {} at their zero position.".format(all_motor_id))
		for motor_id in all_motor_id:
			motor_pose["zero"][str(motor_id)] = b.actuator_pos(motor_id)
			
		save_motor_pose(motor_pose)
		
		for motor_id in all_motor_id:
			b.position_control (motor_pose["zero"][str(motor_id)], 0.5, motor_id)
		
		input("Get your hands away from the robot.")
		
		for motor_id in all_motor_id:
			b.position_control (motor_pose["rest"][str(motor_id)], 0.5, motor_id)
		
		input("Enter to go back to rest.")
		
		for motor_id in all_motor_id:
			b.turn_off(motor_id)
		
	
	elif sys.argv[1] == "check":
		b.init_bus ()
		
		all_motor_id = [int(x) for x in sys.argv[2:]]
		motor_pose = load_motor_pose()
		
		if not check_start(all_motor_id, motor_pose):
			raise NameError("Start position of motor {} is too far from saved rest position. Please re-set rest and zero pose.".format(str(motor_id)))
		
		input("Enter to go to zero pose.")
		for motor_id in all_motor_id:
			b.position_control (motor_pose["zero"][str(motor_id)], 0.5, motor_id)
		
		input("Enter to go to rest pose.")
		for motor_id in all_motor_id:
			b.position_control (motor_pose["rest"][str(motor_id)], 0.5, motor_id)
		
		input("Enter to go to rest pose.")
		for motor_id in all_motor_id:
			b.turn_off(motor_id)
	
	else:
		raise NameError("{} is not a valid argument.".format(sys.argv[1]))
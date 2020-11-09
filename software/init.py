"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
source activate psc
python init.py set 1
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


if __name__ == "__main__":
	if len(sys.argv) < 3:
		raise NameError("Not enough args")
	elif sys.argv[1] == "set":
		
		b.init_bus ()
		
		choosen_motors_id = [int(x) for x in sys.argv[2:]]
		motor_pose = load_motor_pose()
		
		# choosing the rest pose
		for motor_id in chosen_motors_id:
			b.set_zero_torque(motor_id)
		
		
		input("Put motors {} at their rest position.".format(chosen_motors_id))
		for motor_id in choosen_motors_id:
			motor_pose["rest"][str(motor_id)] = b.actuator_pos(motor_id)
		
		input("Put motors {} at their zero position.".format(chosen_motors_id))
		for motor_id in chosen_motors_id:
			motor_pose["zero"][str(motor_id)] = b.actuator_pos(motor_id)
			
		save_motor_pose(motor_pose)
		
		for motor_id in chosen_motors_id:
			b.position_control (motor_pose["zero"][str(motor_id)], 0.5, motor_id)
		
		input("Get your hands away from the robot.")
		
		for motor_id in chosen_motors_id:
			b.position_control (motor_pose["rest"][str(motor_id)], 0.5, motor_id)
		
		input("Enter to go back to rest.")
		
		for motor_id in chosen_motors_id:
			b.turn_off(motor_id)
		
	
	elif sys.argv[1] == "check":
		b.init_bus ()
		
		chosen_motors_id = [int(x) for x in sys.argv[2:]]
		motor_pose = load_motor_pose()
		
		for motor_id in chosen_motors_id:
			start_position = b.actuator_pos(motor_id))
			if abs(start_position - motor_pose["rest"][str(motor_id)]) > 3.14/6/10:
				raise NameError("Start position of motor {} is too far from saved rest position. Please re-set rest and zero pose.".format(str(motor_id)))
		
		print("Going to zero pose.")
		for motor_id in chosen_motors_id:
			b.position_control (motor_pose["zero"][str(motor_id)], 0.5, motor_id)
		
		input("Enter to go to rest pose.")
		for motor_id in chosen_motors_id:
			b.position_control (motor_pose["rest"][str(motor_id)], 0.5, motor_id)
		
		input("Enter to go to rest pose.")
		for motor_id in chosen_motors_id:
			b.turn_off(motor_id)
	
	else:
		raise NameError("{} is not a valid argument.".format(sys.argv[1]))
"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
source activate psc
python init.py set 1
"""

import backend as b
import sys
import json

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
		print("Id of changed motors config", choosen_motors_id)
		motor_pose = load_motor_pose()
		
		print("Setting the rest pose of motors {}.".format(choosen_motors_id))
		for motor_id in choosen_motors_id:
			motor_pose["rest"][str(motor_id)] = b.actuator_pos(motor_id)
		
		input("Put motors {} at their zero position.".format(choosen_motors_id))
		for motor_id in choosen_motors_id:
			motor_pose["zero"][str(motor_id)] = b.actuator_pos(motor_id)
			
		save_motor_pose(motor_pose)
		
		for motor_id in choosen_motors_id:
			print("setting {} to {}".format(str(motor_id), motor_pose["zero"][str(motor_id)]))
			#b.position_control (motor_pose["zero"][str(motor_id)], 0.5, motor_id)
		"""
		time.sleep(0.5)
		motors.goto_rest ()
		time.sleep(0.5)
		motors.disengage ()
		"""
		
	elif sys.argv[2] == "check":
		pass

"""
if __name__ == "__main__":
	if len(sys.argv) < 3:
		raise NameError("Not enough args")
		
	elif sys.argv[1] == "rest":
		if len(sys.argv) < 3:
			raise NameError("Need motors args to operate")
		else:
			motors.check_configuration ()
			motors_id = [int(x) for x in sys.argv[2:]]
			print(motors_id)
			motor_pose = load_motor_pose()
			for id in motors_id:
				motor_pose["rest"][str(id)] = motors.get_abs_pos(id)
			save_motor_pose(motor_pose)
			
			
			
			
		
	elif sys.argv[1] == "zero":
		if len(sys.argv) < 3:
			raise NameError("Need motors args to operate")
		else:
			motors.check_configuration ()
			motors_id = [int(x) for x in sys.argv[2:]]
			print(motors_id)
			motor_pose = load_motor_pose()
			for id in motors_id:
				motor_pose["zero"][str(id)] = motors.get_abs_pos(id)
			save_motor_pose(motor_pose)
	else:
		raise NameError("arg not recognized")
	
	#print(json.dumps({"zero":{}, "rest":{}}))
"""
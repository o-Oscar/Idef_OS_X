import backend as b
import time
import json
import sys


_epsilon = None
_n = 3 #nombre de moteurs
_motors_id = [1, 2, 3] # [1,2,3,4,5,6,7,8,9,10,11,12]
_reduction = [1, 1, 0.8]
_max_vel = 0.5
_origin = None
_rest_pos = None


def load_motor_pose ():
	global _saved_pose
	global _origin
	global _rest_pos
	with open("motor_pose.txt") as f:
		_saved_pose = json.loads(''.join(f.readlines()))
	_origin = [0]*_n
	_rest_pos = [0]*_n
	for i, motor_id in enumerate(_motors_id) :
		_origin[i] = _saved_pose["zero"][str(motor_id)]
		_rest_pos[i] = _saved_pose["rest"][str(motor_id)]
	print(_origin)
	print(_rest_pos)
	
def save_motor_pose ():
	global _saved_pose
	with open("motor_pose.txt", "w") as f:
		f.write(json.dumps(_saved_pose))

def check_configuration ():
	for motor_id in _motors_id :
		pass
	b.init_bus()
	#load_motor_pose()
	return True


def set_origin_to_real():
	global _origin
	print("set_origin_to_real is depreciated : please use config file")
	if _origin is None:
		_origin = [0]*_n
		for i, motor_id in enumerate(_motors_id) :
			_origin[i] = b.actuator_pos(motor_id)
	else :
		raise ErrorName("origin already defined")

	

def position_control():
	vmax = 1
	for motor_id, red in zip (_motors_id, _reduction):
		b.position_control(b.actuator_pos(motor_id), vmax, motor_id)
		



def goto(targ_pos, targ_vel=[_max_vel]*_n, epsilon=0.5):
	if _origin is None:
		raise NameError("non defined origin")
	else :
		global _epsilon
		_epsilon = epsilon
		
		global _targ_pos
		_targ_pos = targ_pos
		
		for motor_id, pos, vel, ori, red in zip (_motors_id, targ_pos, targ_vel, _origin, _reduction) :
			b.position_control (red*pos + ori, red*vel, motor_id)

def goto_rest ():
	for motor_id, pos in zip (_motors_id, _rest_pos) :
		b.position_control (pos, 0.5, motor_id)

def reached_target():
	if _epsilon is None or _targ_pos is None or _origin is None:
		raise NameError("call of reached_target before goto")
	res = True
	for motor_id, pos, ori, red in zip (_motors_id, _targ_pos, _origin, _reduction):
		res = res and ((abs(red*pos + ori - b.actuator_pos(motor_id))) < _epsilon)
	return res

def get_pos ():
	to_return = []
	for motor_id in _motors_id:
		to_return.append(b.actuator_pos(motor_id))
	return to_return

def disengage():
	for motor_id in _motors_id :
		data = b.send_command([0x80, 0, 0, 0, 0, 0, 0, 0],motor_id).data # turns the motor off (CONNECTS the motor phases)
		#data = b.send_command([0xA1, 0, 0, 0, 0, 0, 0, 0],motor_id).data # set torque to zero (UNPLUGS the motor phases)


def all_actuator_pos() :
	if _origin is None :
		raise ErrorName("origin non defined")

	else :
		position = []
		for motor_id, ori, red in zip (_motor_id, _origin, _reduction):
			position.append((b.actuator_pos(motor_id) - ori)/red)
		return(position)





if __name__ == "__main__":
	if len(sys.argv) < 3:
		raise NameError("Not enough args")
		
	elif sys.argv[1] == "rest":
		if len(sys.argv) < 3:
			raise NameError("Need motors args to operate")
		else:
			check_configuration ()
			motors_id = [int(x) for x in sys.argv[2:]]
			for id in motors_id:
				_saved_pose["rest"][str(id)] = b.actuator_pos(id)
			save_motor_pose()
			print(_saved_pose)
			print((get_pos()))
			
		
	elif sys.argv[1] == "zero":
		if len(sys.argv) < 3:
			raise NameError("Need motors args to operate")
		else:
			check_configuration ()
			motors_id = [int(x) for x in sys.argv[2:]]
			for id in motors_id:
				_saved_pose["zero"][str(id)] = b.actuator_pos(id)
			save_motor_pose()
			print(_saved_pose)
			print((get_pos()))
	else:
		raise NameError("arg not recognized")

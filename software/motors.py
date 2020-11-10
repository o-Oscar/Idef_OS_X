import backend as b
import time
import json
import sys
import init


_epsilon = None
_motors_id = [6] # [1,2,3,4,5,6,7,8,9,10,11,12]
_n = len(_motors_id) #nombre de moteurs
_reduction = [1] # [1, 1, 30/24] * 4
_max_vel = 0.5
_origin = None
_rest_pos = None
_motor_pose = None



def check_configuration ():
	global _motor_pose
	global _origin
	global _rest_pos
	
	_motor_pose = init.load_motor_pose()
	b.init_bus()
	
	if not init.check_start (_motors_id, _motor_pose):
		raise NameError("Start position of motor {} is too far from saved rest position. Please re-set rest and zero pose.".format(str(motor_id)))
		return False
    
	_origin = [0]*_n
	_rest_pos = [0]*_n
	for i, motor_id in enumerate(_motors_id) :
		_origin[i] = _motor_pose["zero"][str(motor_id)]
		_rest_pos[i] = _motor_pose["rest"][str(motor_id)]
		# set PID : angle_kp, angle_ki, speed_kp, speed_ki, current_kp, current_ki
		b.set_pid(motor_id, 100, 100, 50, 40, 50, 50)
		
	return True


"""
# DEPRECIATED
def set_origin_to_real():
	global _origin
	print("set_origin_to_real is depreciated : please use config file")
	if _origin is None:
		_origin = [0]*_n
		for i, motor_id in enumerate(_motors_id) :
			_origin[i] = b.actuator_pos(motor_id)
	else :
		raise ErrorName("origin already defined")
"""
	

"""
# Unused
def position_control():
	vmax = 1
	for motor_id, red in zip (_motors_id, _reduction):
		b.position_control(b.actuator_pos(motor_id), vmax, motor_id)
"""	



def goto(targ_pos, targ_vel=[_max_vel]*_n, epsilon=0.5):
	if _origin is None:
		raise NameError("non defined origin")
	else :
		global _epsilon
		_epsilon = epsilon
		
		global _targ_pos
		_targ_pos = targ_pos
		
		for motor_id, pos, vel, ori, red in zip (_motors_id, targ_pos, targ_vel, _origin, _reduction) :
			b.position_control (motor_id, red*pos + ori, red*vel)

def goto_rest ():
	for motor_id, pos in zip (_motors_id, _rest_pos) :
		b.position_control (motor_id, pos, 0.5)

def reached_target():
	if _epsilon is None or _targ_pos is None or _origin is None:
		raise NameError("call of reached_target before goto")
	res = True
	for motor_id, pos, ori, red in zip (_motors_id, _targ_pos, _origin, _reduction):
		res = res and ((abs(red*pos + ori - b.actuator_pos(motor_id))) < _epsilon)
	return res

def get_abs_pos ():
	to_return = []
	for motor_id in _motors_id:
		to_return.append(b.actuator_pos(motor_id))
	return to_return

def get_pos () :
	if _origin is None :
		raise ErrorName("origin non defined")

	else :
		position = []
		for motor_id, ori, red in zip (_motor_id, _origin, _reduction):
			position.append((b.actuator_pos(motor_id) - ori)/red)
		return(position)


def disengage():
	for motor_id in _motors_id :
		b.turn_off(motor_id)






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

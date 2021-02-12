import backend as b
import time
import json
import sys
import init


_epsilon = None
_motors_id = [1,2,3,4,5,6,7,8,9,10,11,12]
_n = len(_motors_id) #nombre de moteurs
_reduction = [1, 1, 24/30] * 4
_origin = None
_rest_pos = None
_motor_pose = None



def check_configuration ():
	global _motor_pose
	global _origin
	global _rest_pos
	
	b.init_bus()
	
	_motor_pose = init.load_motor_pose()
	_motor_pose = init.check_start (_motors_id, _motor_pose)
	"""
	if True:
		raise NameError("Start position of motor {} is too far from saved rest position. Please re-set rest and zero pose.".format(str(_motors_id)))
		return False
	"""
	_origin = [0]*_n
	_rest_pos = [0]*_n
	for i, motor_id in enumerate(_motors_id) :
		_origin[i] = _motor_pose["zero"][str(motor_id)]
		_rest_pos[i] = _motor_pose["rest"][str(motor_id)]
		
		# set PID : angle_kp, angle_ki, speed_kp, speed_ki, current_kp, current_ki
		# carefull with this one : extreamly powerfull : b.set_pid(motor_id, 100, 0, 50, 40, 50, 50)
		b.set_pid(motor_id, 4*2, 0, 50, 0, 50, 50)
		
	return True

def set_lite_pid ():
	for i, motor_id in enumerate(_motors_id):
		#b.set_pid(motor_id, 6, 0, 100, 0, 50, 50)
		if i%3 == 2:
			#b.set_pid(motor_id, 6, 0, 62, 0, 50, 50)
			b.set_pid(motor_id, 6, 0, 50, 0, 50, 50)
		else:
			b.set_pid(motor_id, 6, 0, 50, 0, 50, 50)

def set_strong_pid ():
	for i, motor_id in enumerate(_motors_id):
		b.set_pid(motor_id, 4*2, 0, 50, 40, 50, 50)

def position_control (targ_pos):
	for motor_id, pos, ori, red in zip (_motors_id, targ_pos, _origin, _reduction) :
		b.position_control (motor_id, red*pos + ori)

def position_control_at_speed (targ_pos, max_speed):
	for motor_id, pos, vel, ori, red in zip (_motors_id, targ_pos, max_speed, _origin, _reduction) :
		b.position_control_at_speed (motor_id, red*pos + ori, red*vel)

def goto_rest ():
	dt = 2
	for motor_id, targ_pos in zip (_motors_id, _rest_pos) :
		cur_pos = b.actuator_pos(motor_id)
		#speed = max((10, abs(cur_pos-targ_pos)/dt))
		speed = abs(cur_pos-targ_pos)/dt
		b.position_control_at_speed (motor_id, targ_pos, speed)

last_fetched_actuator_pos = None
def get_pos (fetch=True) :
	global last_fetched_actuator_pos
	
	if _origin is None :
		raise ErrorName("origin non defined")

	else :
		if not fetch and last_fetched_actuator_pos is None:
			raise ErrorName("You have to fetch the position of the motors at least once")
		elif fetch:
			last_fetched_actuator_pos = []
			for motor_id, ori, red in zip (_motors_id, _origin, _reduction):
				last_fetched_actuator_pos.append((b.actuator_pos(motor_id) - ori)/red)
		
		return last_fetched_actuator_pos

def get_speeds ():
	speeds = []
	for motor_id, red in zip (_motors_id, _reduction):
		raw_speed, raw_torque = actuator_info(motor_id)
		speeds.append(raw_speed/red)
	return speeds

def disengage():
	for motor_id in _motors_id :
		b.turn_off(motor_id)

if __name__ == "__main__":
	b.init_bus()
	disengage()
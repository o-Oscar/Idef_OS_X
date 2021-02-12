import can
import numpy as np
import struct

_bus = None

def init_bus(channel='can0', bustype='socketcan_native') :
	global _bus
	if _bus is None :
		print("Creating can bus image.")
		_bus = can.interface.Bus(channel=channel, bustype=bustype)



def send_command(command, motor_id) :
	if _bus is None :
		raise NameError("_bus not initialised")
	
	if not isinstance(command, list):
		raise NameError("Command {} is in wrong format.".format(str(command)))
		
	msg = can.Message(arbitration_id=0x140+motor_id,
					  data=command,
					  is_extended_id=False)
	"""
	try:
		_bus.send(msg)
	except can.CanError:
		print("Can message not sent !!! Good luck for recovery !")
	"""
	_bus.send(msg)
	ret = _bus.recv()
	
	return ret


def set_pid (motor_id, angle_kp, angle_ki, speed_kp, speed_ki, current_kp, current_ki):
	send_command([0x31, 0, angle_kp, angle_ki, speed_kp, speed_ki, current_kp, current_ki], motor_id)


def position_control(motor_id, rad_pos) : #position : rad
	motor_pos = int(rad_pos * 36000 / (2*np.pi) * 6)
	motor_speed = int(10 * 360 * 6)
	
	bytes_pos = motor_pos.to_bytes(4, byteorder="little", signed=True)
	bytes_speed = motor_speed.to_bytes (2, byteorder="little", signed=True)
	
	data = send_command([0xA4, 0, bytes_speed[0], bytes_speed[1], bytes_pos[0], bytes_pos[1], bytes_pos[2], bytes_pos[3]],motor_id).data
	

def position_control_at_speed (motor_id, rad_pos, rad_speed) : #position : rad | speed : rad.s-1
	motor_pos = int(rad_pos * 36000 / (2*np.pi) * 6)
	motor_speed= min(1000, int(rad_speed * 360 / (2*np.pi) * 6))
	
	bytes_pos = motor_pos.to_bytes(4, byteorder="little", signed=True)
	bytes_speed = motor_speed.to_bytes (2, byteorder="little", signed=True)
	
	data = send_command([0xA4, 0, bytes_speed[0], bytes_speed[1], bytes_pos[0], bytes_pos[1], bytes_pos[2], bytes_pos[3]],motor_id).data
	
	
def actuator_pos (motor_id): # actuator pos in rad nice and clean
	data=send_command([0x92, 0, 0, 0, 0, 0, 0, 0], motor_id).data
	data[0] = 0
	return struct.unpack("<q", data)[0] * 2*np.pi / 2**8 / 36000 / 6
	

def actuator_info (motor_id): # actuator pos in rad nice and clean
	data=send_command([0x9C, 0, 0, 0, 0, 0, 0, 0], motor_id).data
	temp = struct.unpack("<b", data[1:2])[0]
	torque = struct.unpack("<h", data[2:4])[0]
	speed = struct.unpack("<h", data[4:6])[0] * 2*np.pi / 360 / 6
	encoder = struct.unpack("<h", data[6:8])[0]
	return temp, speed, torque, encoder

	
def turn_off (motor_id): # turns the motor off (CONNECTS the motor phases)
	data = send_command([0x80, 0, 0, 0, 0, 0, 0, 0],motor_id).data
	
def set_zero_torque (motor_id): # set torque to zero (UNPLUGS the motor phases)
	data = send_command([0xA1, 0, 0, 0, 0, 0, 0, 0],motor_id).data
	




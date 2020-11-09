import can
import time
import numpy as np
import struct



_bus = None

def init_bus(channel='can0', bustype='socketcan_native') :
    global _bus
    if _bus is None :
		print("Creating can bus image.")
        _bus = can.interface.Bus(channel=channel, bustype=bustype)



def send_command(command,motor_id) :
    
    if _bus is None :
        raise NameError("_bus not initialised")
    
    
    msg = can.Message(arbitration_id=0x140+motor_id,
                      data=command,
                      is_extended_id=False)
	"""
    try:
        _bus.send(msg)
    except can.CanError:
        print("Can message not sent !!! Good luck for recovery !")
	"""
	
	bus.send(msg)
    return(_bus.recv())





def position_control(rad_pos,rad_speed,motor_id) : #position : rad ; speed : rad.s-1
    motor_pos = int(rad_pos * 36000 / (2*np.pi) * 6)
    motor_speed= int(rad_speed * 360 / (2*np.pi) * 6)
    
    bytes_pos = motor_pos.to_bytes(4, byteorder="little", signed=True)
    bytes_speed = motor_speed.to_bytes (2, byteorder="little", signed=True)
    
    data = send_command([0xA4, 0, bytes_speed[0], bytes_speed[1], bytes_pos[0], bytes_pos[1], bytes_pos[2], bytes_pos[3]],motor_id).data

"""
# DEPRECIATED
def motor_pos (motor_id): # motor position in turn
    data = send_command([0x90, 0, 0, 0, 0, 0, 0, 0],motor_id).data
    first = data[2]
    sec = data[3]
    return (first + 2**8 * sec)/ (2**16)
"""

def actuator_pos (motor_id): # actuator pos in rad nice and clean
    data=send_command([0x92, 0, 0, 0, 0, 0, 0, 0], motor_id).data
    data[0] = 0
    return struct.unpack("<q", data)[0] * 2*np.pi / 2**8 / 36000 / 6
    
def turn_off (motor_id): # turns the motor off (CONNECTS the motor phases)
	data = b.send_command([0x80, 0, 0, 0, 0, 0, 0, 0],motor_id).data
	
def set_zero_torque (motor_id): # set torque to zero (UNPLUGS the motor phases)
	data = b.send_command([0xA1, 0, 0, 0, 0, 0, 0, 0],motor_id).data
	









import backend as b
import time


_epsilon = None
_n = 3 #nombre de moteurs
_motors_id = [2, 1, 3] # [1,2,3,4,5,6,7,8,9,10,11,12]

def check_configuration ():
    for motor_id in _motors_id :
        pass
    b.init_bus()
    return True


def position_control():
    vmax = 1
    for motor_id in _motors_id:
        b.position_control(b.actuator_pos(motor_id), vmax, motor_id)
        



def goto(target_pos, epsilon=0.5, max_speed=2):
    global _epsilon
    _epsilon = epsilon
    
    global _target_pos
    _target_pos = target_pos
    
    for motor_id, target in zip (_motors_id, target_pos) :
        b.position_control (target, max_speed, motor_id)


def reached_target():
    if _epsilon is None or _target_pos is None:
        raise NameError("call of reached_target before goto")
    res = True
    for motor_id, target in zip (_motors_id, _target_pos):
        res = res and (abs(target - b.actuator_pos(motor_id)) < _epsilon)
    return res

def disengage():
    for motor_id in _motors_id :
        #data=send_command([0x80, 0, 0, 0, 0, 0, 0, 0],2).data # turns the motor off (CONNECTS the motor phases)
        data = b.send_command([0xA1, 0, 0, 0, 0, 0, 0, 0],motor_id).data # set torque to zero (UNPLUGS the motor phases)

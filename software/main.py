"""
TODO :
- un rapport de réduction (fixé) pour chaque moteur
"""

"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
source activate psc
python main.py
"""

import motors
import kinematics
import time
import numpy as np

def calc_motor_targ (start_pos, end_pos, dt, lamb=0.1):
    delta = end_pos-start_pos
    targ_pos = end_pos + delta*lamb
    targ_vel = delta/dt
    return targ_pos, abs(targ_vel)

T = 0.7
def pos_at (t):
	return [0.5+np.sin(t/T*2*np.pi)*0.4, 0.5, 0.2+np.cos(t/T*2*np.pi)*0.2]


all_target_kin = [(kinematics.motor_pos ([0.5, 0.5, 0] * 4), 1),
				(kinematics.motor_pos (pos_at(0) * 4), 1)]

N = 120*2
for i in range(1, N+1):
	all_target_kin.append((kinematics.motor_pos (pos_at(i/60) * 4), 1/30))
	
all_target_kin += [(kinematics.motor_pos ([0.5, 0.5, 0] * 4), 1),
				(np.zeros([12]), 1)]



"""
x_start = 1
x_end = 0.2
dt = 2
N = 60

all_target_kin = [(kinematics.motor_pos ([0.5, 0.5, 0] * 4), 1),
				(kinematics.motor_pos ([x_start, 0.5, 0] * 4), 1)]

for i in range(1, N+1):
	all_target_kin.append((kinematics.motor_pos ([x_start + i/N*(x_end-x_start), 0.5, 0] * 4), dt/N))
	
all_target_kin += [(kinematics.motor_pos ([0.5, 0.5, 0] * 4), 1),
				(np.zeros([12]), 1)]
"""

if __name__ == "__main__":
	
	if not motors.check_configuration ():
		print("error in reading config")

	else:
		print("config is matching")
		
		motors.set_origin_to_real()
		
		motors.position_control()
		
		# --- going slowly to the starting pos ---
		start_pos = np.zeros([12])
		end_pos = start_pos
		
		start = time.time()
		for kin, dt in all_target_kin:
			start_pos = end_pos
			end_pos = kin
			#end_pos[0] = 0
			#end_pos[1] = 0
			targ_pos, targ_vel = calc_motor_targ (start_pos, end_pos, dt, lamb=0.2)
			targ_vel = np.maximum(targ_vel, 0.01)
			#print((start_pos-end_pos)[1:3])
			#print(np.maximum(targ_vel, 0.01)[1:3])
			motors.goto(targ_pos, targ_vel=targ_vel, epsilon=0.02)
			
			time.sleep(max(dt-0.004, 0))
			#print(dt, time.time()-start)
			start = time.time()
		
		motors.disengage ()
		

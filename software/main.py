"""
TODO :
- une vitesse max par moteur pour la fonction 
- un "zéro" qu'on peut choisir en lançant une fonction set_origin_to_real (forcer un seul choix de zéro, empêcher les commandes mouvements tant qu'on a pas de zéro)
"""


import motors
import kinematics
import time


def calc_motor_targ (start_pos, end_pos, dt, lamb=0.1):
    delta = end_pos-start_pos
    targ_pos = end_pos + delta*lamb
    targ_vel = delta/dt
    return targ_pos, targ_vel

if __name__ == "__main__" :
	
	if not motors.check_configuration ():
		print("error in reading config")

	else:
		print("config is matching")
		
		motors.set_origin_to_real()
		
		motors.position_control()
		
		# --- going slowly to the starting pos ---
		
		start_pos = np.zeros([12])
		end_pos = kinematics.motor_pos ([0.5] * 12)
		dt = 1
		targ_pos, targ_vel = calc_motor_targ (start_pos, end_pos, dt, lamb=0)
		
		motors.goto(targ_pos, targ_vel=targ_vel, epsilon=0.1, max_speed=2)
		
		while not motors.reached_target():
			print("going to zero")
			time.sleep(0.3)
		print("reached startig pos")
		
		# --- going slowly zero ---
		
		start_pos = end_pos
		end_pos = np.zeros([12])
		dt = 1
		targ_pos, targ_vel = calc_motor_targ (start_pos, end_pos, dt, lamb=0)
		
		motors.goto(targ_pos, targ_vel=targ_vel, epsilon=0.1, max_speed=2)
		
		while not motors.reached_target():
			print("going to zero")
			time.sleep(0.3)
		print("reached startig pos")
		
		motors.disengage ()
		

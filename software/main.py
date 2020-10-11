
import motors
import kinematics


if __name__ == "__main__" and False:
	
	if not motors.check_configuration ():
		print("error in reading config")
	
	else:
		print("config is matching")
		
		motor.position_control()
		
		# --- going slowly to the starting pos --- 
		motors.goto(kinematics.motor_pos ([0.5] * 12), epsilon=0.1)
		while not motors.reached_target():
			print("going to zero")
			time.sleep(0.3)
		print("reached startig pos")
		
		
		# --- going slowly to the first pos, waiting 1s to do the move ---
		motors.goto(kinematics.motor_pos ([0] * 12), epsilon=0.1)
		print("going to front")
		time.sleep(1)
		
		
		# --- going fast to the back pos ---
		motors.goto(kinematics.motor_pos ([1] * 12), epsilon=0.1, max_speed=2.)
		while not motors.reached_target():
			print("going to back")
			time.sleep(0.3)
		print("reached back")
		
		
		# --- going back slowly to the starting pos ---
		motors.goto(kinematics.motor_pos ([0.5] * 12), epsilon=0.1)
		while not motors.reached_target():
			print("going to zero")
			time.sleep(0.3)
		print("reached end pos")
	
		motor.disengage ()
		
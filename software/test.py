

"""
sudo ip link set can0 up type can bitrate 1000000
cd /home/pi/psc/IDEFX/software
source activate psc
python test.py

sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000

ip -details -statistics link show can0
"""

import backend as b
import time

motor_id = 12

b.init_bus()
print("reading the actuator pos :")
init_pos = b.actuator_pos(motor_id)
print(init_pos)
#b.position_control(init_pos, 0.5, motor_id)


import can
import time

def send_one(command):

    # this uses the default configuration (for example from the config file)
    # see https://python-can.readthedocs.io/en/stable/configuration.html
    bus=can.interface.Bus(channel='can0', bustype='socketcan_native')
    # Using specific buses works similar:
    # bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=250000)
    # bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
    # bus = can.interface.Bus(bustype='ixxat', channel=0, bitrate=250000)
    # bus = can.interface.Bus(bustype='vector', app_name='CANalyzer', channel=0, bitrate=250000)
    # ...

    msg = can.Message(arbitration_id=0x142,
                      data=command,
                      is_extended_id=False)
    print(msg)
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")
    print(bus.recv())


"""
Fonctions de conversion
"""

def speed_to_for_bytes(speed) : #speed en tours par seconde
    speed_very_high_byte=int(6*36000*speed//(2**24))
    speed_high_byte=int((6*36000*speed-speed_very_high_byte*2**24)//(2**16))
    speed_low_byte=int((6*36000*speed-speed_very_high_byte*2**24-speed_high_byte*2**16)//(2**8))
    speed_very_low_byte=int((6*36000*speed-speed_very_high_byte*2**24-speed_high_byte*2**16-speed_low_byte*2**8)//1)
    return speed_very_low_byte, speed_low_byte, speed_high_byte, speed_very_high_byte

#def byte_to_speed(speed_low_byte, speed_high_byte)

def speed_to_two_bytes(speed) : #speed en tours par seconde
    speed_high_byte=int(6*360*speed//(2**8))
    speed_low_byte=int((6*360*speed-speed_high_byte*2**8)//1)
    return speed_low_byte, speed_high_byte


def position_to_for_bytes(position) : #en tours
    position_very_high_byte=int(6*36000*position//(2**24))
    position_high_byte=int((6*36000*position-position_very_high_byte*2**24)//(2**16))
    position_low_byte=int((6*36000*position-position_very_high_byte*2**24-position_high_byte*2**16)//(2**8))
    position_very_low_byte=int((6*36000*position-position_very_high_byte*2**24-position_high_byte*2**16-position_low_byte*2**8)//1)
    
    return position_very_low_byte, position_low_byte, position_high_byte, position_very_high_byte



def position_to_two_bytes(position) : #position en tours
    position_high_byte=int(6*360*position//(2**8))
    position_low_byte=int((6*360*position-position_high_byte*2**8)//1)
    return position_low_byte, position_high_byte

"""
Fonctions de commandes
"""




### Read PID parameter command



def send_command(command,ide) :
    
    bus=can.interface.Bus(channel='can0', bustype='socketcan_native')
    
    msg = can.Message(arbitration_id=0x140+ide,
                      data=command,
                      is_extended_id=False)
    print(msg)
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")
    print(bus.recv())
    return(bus.recv())




def read_PID_parameter(ide) :
    
    retour=send_command([0x30, 0, 0, 0, 0, 0, 0, 0],ide).data
    
    Position_loop_Kp=retour[2]
    Position_loop_Ki=retour[3]
    Speed_loop_Kp=retour[4]
    Speed_loop_Ki=retour[5]
    Torque_loop_Kp=retour[6]
    Torque_loop_Ki=retour[7]
    
    #Impression des données
    print("The Position Loop Kp is "+ str(Position_loop_Kp))
    print("The Position Loop Ki is " +str (Position_loop_Ki))
    print("The Speed Loop Kp is " + str(Speed_loop_Kp))
    print("The Speed Loop Ki is " + str(Speed_loop_Ki))
    print("The Torque Loop Kp is " + str(Torque_loop_Kp))
    print("The Torque Loop Ki is " + str(Torque_loop_Ki))
    
    return retour


### Write PID to RAM parameter command

def write_to_RAM_parameter(Position_loop_Kp,Position_loop_Ki,
                           Speed_loop_Kp,Speed_loop_Ki,
                           Torque_loop_Kp,Torque_loop_Ki,
                           ide) :
    
    send_command([0x31, 0, Position_loop_Kp, Position_loop_Ki, Speed_loop_Kp, Speed_loop_Ki, Torque_loop_Kp, Torque_loop_Ki],ide).data
  




### Write PID to ROM parameter command

def write_to_ROM_parameter(Position_loop_Kp,Position_loop_Ki,
                           Speed_loop_Kp,Speed_loop_Ki,
                           Torque_loop_Kp,Torque_loop_Ki,
                           ide) :
    
    send_command([0x32, 0, Position_loop_Kp, Position_loop_Ki, Speed_loop_Kp, Speed_loop_Ki, Torque_loop_Kp, Torque_loop_Ki],ide).data



### Read acceleration data command




### Write acceleration data to RAM command




### Write encoder offset command





### Write current position to ROM as motor zero position command
    



### Read multi turns angle command




### Read single circle angle command
    


### Motor off command

def motor_off(ide) :
    send_command([0x80, 0, 0, 0, 0, 0, 0, 0],ide)



### Motor sleep command

def motor_sleep(ide) :
    send_command([0x81, 0, 0, 0, 0, 0, 0, 0],ide)




### Motor running command
    
def motor_running(ide) :
    send_command([0x88, 0, 0, 0, 0, 0, 0, 0],ide)




### Read motor status 1 and error flag command




### Clear motor error flag




### Read motor status 2



### Read motor status 3




### Speed control command
    
def speed_control (speed,ide) :
    speed_very_low_byte, speed_low_byte, speed_high_byte, speed_very_high_byte=speed_to_for_bytes(speed)
    retour=send_command([0xA2, 0, 0, 0, speed_very_low_byte, speed_low_byte, speed_high_byte, speed_very_high_byte],ide)
    print("reponse :")
    print(retour.data)


### Position control command 4

def position_control(position, speed,ide) :
    position_very_low_byte, position_low_byte, position_high_byte, position_very_high_byte=position_to_for_bytes(position)
    speed_low_byte, speed_high_byte=speed_to_two_bytes(speed)
    
    retour=send_command([0xA4, 0, speed_low_byte, speed_high_byte, position_very_low_byte, position_low_byte, position_high_byte, position_very_high_byte],ide).data
    
    temperature=retour[1]
    
    torque_low_byte=retour[2]
    torque_high_byte=retour[3]
    
    speed_low_byte=retour[4]
    speed_high_byte=retour[5]
    
    position_low_byte=retour[6]
    position_low_byte=retour[7]
    




if __name__ == '__main__':
    read_PID_parameter(2)
    write_to_RAM_parameter(100,100,100,100,100,100,2)
    read_PID_parameter(2)
    position_control(0, 1, 2)


#     position_control(1, 1, 2)
#     time.sleep(2)
#     position_control(0, 1, 2)
#     time.sleep(2)
#     motor_off(2)









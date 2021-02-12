import time
import asyncio
import websockets
import pygame
import time
import json


async def send_stuff (name):
	uri = "ws://localhost:8765"
	async with websockets.connect(uri) as websocket:
		#name = input("What's your name? ")
		await websocket.send(name)
		print(f"> {name}")

def update_remote_cmd (cmd):
	msg = json.dumps(cmd)
	print("sending : '{}'".format(msg))
	asyncio.get_event_loop().run_until_complete(send_stuff(msg))
	

def hello():
	uri = "ws://localhost:8765"
	last_time = 0
	data = {}
	
	pygame.init()
	pygame.joystick.init()
	
	while 1:

		pygame.event.get()
		joystick = pygame.joystick.Joystick(0)
		joystick.init()
		for i in range(joystick.get_numaxes()):
			data["axis_" + str(i)] = joystick.get_axis(i)
		for i in range(joystick.get_numbuttons()):
			data["button_"+str(i)] = joystick.get_button(i)
		
		if time.time() - last_time > 0.3:
			update_remote_cmd(data)
			last_time = time.time()

if __name__ == "__main__":
	hello()

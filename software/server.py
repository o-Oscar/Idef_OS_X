"""
ssh -L 8765:127.0.0.1:8765 pi@192.168.1.40

"""

from threading import Thread
import asyncio
import websockets
import time
import queue
import json

q = queue.Queue()
cmd = {}
last_cmd_update = -1

async def reading_socket(websocket, path):
	name = await websocket.recv()
	#print(f"< {name}")
	q.put(name)
	

def server_side ():
	global loop
	loop = asyncio.new_event_loop()
	asyncio.set_event_loop(loop)
	
	socket = websockets.serve(reading_socket, "localhost", 8765)
	loop.run_until_complete(socket)
	print("starting server")
	loop.run_forever()
	print("closing server")

def start_server ():
	t = Thread(target = server_side, daemon=True)
	t.start()

def stop_server ():
	loop.call_soon_threadsafe(loop.stop)

def update_cmd ():
	try :
		last_msg = q.get(block=False)
		cmd.update(json.loads(last_msg))
		last_cmd_update = time.time()
	except queue.Empty:
		pass
		#print("empty")
	return cmd

if __name__ == "__main__":
	start_server()

	start = time.time()
	while time.time() - start < 10:
		time.sleep(0.3)
		print(update_cmd())
	
	stop_server()
	
	time.sleep(0.3)
	

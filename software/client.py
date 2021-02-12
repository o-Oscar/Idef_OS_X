import asyncio
import websockets
import json
import sys


async def send_stuff (name):
	uri = "ws://localhost:8765"
	async with websockets.connect(uri) as websocket:
		#name = input("What's your name? ")
		await websocket.send(name)
		print(f"> {name}")

if __name__ == "__main__":
	if len(sys.argv) > 2:
		key = sys.argv[1]
		value = sys.argv[2]
		
		# --- getting value the right type ---
		type_found = False
		to_test = [int, float]
		for func in to_test:
			if not type_found:
				try:
					value = func(value)
					type_found = True
				except ValueError:
					pass
		cmd = json.dumps({key:value})
		asyncio.get_event_loop().run_until_complete(send_stuff(cmd))
	else:
		print("I need args to send.")
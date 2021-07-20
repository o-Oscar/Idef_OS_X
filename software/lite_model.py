import tflite_runtime.interpreter as tflite
#import tensorflow.lite as tflite
import numpy as np


def load (path):
	global interpreter
	global input_details
	global output_details
	
	# Load the TFLite model and allocate tensors.
	interpreter = tflite.Interpreter(model_path=path.format('model.tflite'))
	interpreter.allocate_tensors()
	# Get input and output tensors.
	input_details = interpreter.get_input_details()
	output_details = interpreter.get_output_details()

hidden_state = [np.zeros((1, 128), dtype=np.float32), np.zeros((1, 128), dtype=np.float32)]
"""
def step (obs):
	inp = (('serving_default_main_input:0', obs.astype(np.float32)), ('serving_default_hidden_0:0', hidden_state[0]), ('serving_default_hidden_1:0', hidden_state[1]))
	out = raw_step(inp)
	
	if 'StatefulPartitionedCall:0' in out:
		hidden_state[0] = out['StatefulPartitionedCall:0']
	if 'StatefulPartitionedCall:1' in out:
		hidden_state[1] = out['StatefulPartitionedCall:1']
	print(out.keys())
	return out['StatefulPartitionedCall:2']
	
"""
def step (obs):
	inp = (('serving_default_main_input:0', obs.astype(np.float32)), ('serving_default_hidden_0:0', hidden_state[0]), ('serving_default_hidden_1:0', hidden_state[1]))
	out = raw_step(inp)
	ret = out['StatefulPartitionedCall:0']
	# return np.minimum(np.maximum(ret, -1), 1)
	return ret

def raw_step (obs):
	for name, arr in obs:
		for detail in input_details:
			if name == detail['name']:
				interpreter.set_tensor(detail['index'], arr)
	interpreter.invoke()
	
	to_return = {}
	for detail in output_details:
		#print(detail['name'])
		to_return[detail['name']] = interpreter.get_tensor(detail['index'])
	
	return to_return

	
def step_old (obs):
	interpreter.set_tensor(input_details[0]['index'], obs.astype(np.float32))
	interpreter.invoke()
	act = interpreter.get_tensor(output_details[0]['index'])
	
	return act
	
if __name__ == "__main__":
	load("models/exp_2/model.tflite")
	print(step(np.load("o.npy")))
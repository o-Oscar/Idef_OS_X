
import tensorflow as tf
from tensorflow.keras import layers
import numpy as np
import time
import obs_parser
from actor import SimpleActor, MixtureOfExpert, LSTMActor
import os

path = os.getcwd() + "\\models\\expert_soft\\{}"

def create_model ():
	env = obs_parser.Env()
	actor_type = "simple"
	#path = os.getcwd() + "/models/expert_0.5ms/{}"
	
	if actor_type=="mix":
		primitives = [SimpleActor(env, inp_dim=1) for i in range(2)]
		actor = MixtureOfExpert(env, primitives, debug=True)
	elif actor_type == "simple":
		actor = SimpleActor(env, inp_dim=1)
	elif actor_type == "lstm":
		actor = LSTMActor(env)
	
	actor.load(path)
	
	return actor.model


model = create_model ()
model.summary()
tf.saved_model.save(model, path.format("saved_model"))
"""
# test model
for i in range(10):
	start = time.time()
	inp = np.random.random_sample((1,3))
	out = model(inp)
	print(out.shape)
	print(time.time()-start)
"""
converter = tf.lite.TFLiteConverter.from_saved_model(path.format("saved_model")) # path to the SavedModel directory
tflite_model = converter.convert()

with open(path.format('model.tflite'), 'wb') as f:
	f.write(tflite_model)
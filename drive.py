import argparse
import base64
import json
import cv2

import numpy as np
import socketio
import flask
import eventlet
import eventlet.wsgi
import time
from PIL import Image
from PIL import ImageOps
from flask import Flask, render_template
from io import BytesIO
import vidi
import io

#define the file direcory
features_directory = './training_data/'

sio = socketio.Server()
app = Flask(__name__)
prev_image_array = None
sample = None
control = None

class SimplePIController:
    def __init__(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.set_point = 0.
        self.error = 0.
        self.integral = 0.

    def set_desired(self, desired):
        self.set_point = desired

    def update(self, measurement):
        # proportional error
        self.error = self.set_point - measurement

        # integral error
        self.integral += self.error

        return self.Kp * self.error + self.Ki * self.integral

controller = SimplePIController(0.2, 0.004)
set_speed = 15
controller.set_desired(set_speed)

@sio.on('telemetry')
def telemetry(sid, data):
	global throttle
	global sample
	global image
	global img
	# The current steering angle of the car
	steering_angle = float(data["steering_angle"])
	# The current throttle of the car
	throttle = float(data["throttle"])
	# The current speed of the car
	speed = float(data["speed"])
	# The current image from the center camera of the car
	imgString = data["image"]
	image = Image.open(BytesIO(base64.b64decode(imgString)))
	image.save("temp.PNG")
	#image_array = np.asarray(image)
	#transformed_image_array = image_array[None, :, :, :]

	# resize the image
	#transformed_image_array = (
	#cv2.resize((cv2.cvtColor(transformed_image_array[0], cv2.COLOR_RGB2HSV))[:, :, 1], (32, 16))).reshape(1, 16, 32, 1)

	# This model currently assumes that the features of the model are just the images. Feel free to change this.
	##steering_angle = 1.3 * float(model.predict(transformed_image_array, batch_size=1))
	
	img = control.load_image("temp.PNG")
	sample = control.process(img, ws_name = "EndtoEndSelfDriving01.vrws")
	img = control.free_image(img)
	analyze = sample.markings['Classify']
	views = analyze['views']
	tag_view = views[0]['tag']
	
	max_value = float(0)
	max_key = int(0)
	for iDict in tag_view:
		kList = [ik for ik in iDict.keys()]
		vList = [iv for iv in iDict.values()]
		# print('k = ', kList)
		# print('v = ', vList)
		if max_value < float(vList[0]):
			max_value = float(vList[0])
			max_key = int(kList[0])
	print('max_v = ', max_value, 'max_k = ', max_key)
	steering_angle = 0.1 * float(max_key - 50)
	
	throttle = controller.update(float(speed))
	if (throttle < float(0.0)):
		throttle = 0.0
	if (throttle > 1.0):
		throttle = 1.0
		
	print('Steering angle =', '%5.2f' % (float(steering_angle)), 'Throttle =', '%.2f' % (float(throttle)), 'Speed  =',
		  '%.2f' % (float(speed)))
	send_control(steering_angle, throttle)

@sio.on('connect')
def connect(sid, environ):
	print("connect ", sid)
	send_control(0, 0)

def send_control(steering_angle, throttle):
	sio.emit("steer", data={
		'steering_angle': steering_angle.__str__(),
		'throttle': throttle.__str__()
	}, skip_sid=True)

if __name__ == '__main__':
	control = vidi.Runtime()
	control.initialize(gpu_mode=vidi.GPUMode.single)
	devices = control.list_compute_devices()
	control.open_workspace_from_file(ws_name="EndtoEndSelfDriving01.vrws", ws_path="EndtoEndSelfDriving01.vrws")
	
	# wrap Flask application with engineio's middleware
	app = socketio.Middleware(sio, app)

	# deploy as an eventlet WSGI server
	eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
	

	
	
	

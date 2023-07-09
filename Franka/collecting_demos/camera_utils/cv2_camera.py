import cv2 
import numpy
import time

def gather_cv2_cameras(max_ind=20):
	all_cv2_cameras = []
	for i in range(max_ind):
		cap = cv2.VideoCapture(i)
		if cap.read()[0]:
			camera = CV2Camera(cap)
			all_cv2_cameras.append(camera)
	return all_cv2_cameras

class CV2Camera:
	def __init__(self, cap):
		self._cap = cap
		self._serial_number = 'cv2_camera' #temporary

	def read_camera(self, enforce_same_dim=False):
		# Get a new frame from camera
		retval, frame = self._cap.read()
		if not retval: return None

		read_time = time.time()
		img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		img = cv2.resize(img, dsize=(100, 100), interpolation=cv2.INTER_AREA)

		dict_1 = {'array': img, 'shape': img.shape, 'type': 'rgb',
			'read_time': read_time, 'serial_number': self._serial_number + '/left'}
		return [dict_1]

	def disable_camera(self):
		self._cap.release()

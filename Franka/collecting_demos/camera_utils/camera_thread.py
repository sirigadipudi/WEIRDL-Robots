import threading
import numpy as np
import cv2
from copy import deepcopy

class CameraThread:
	def __init__(self, camera):
		self._camera = camera
		self._latest_feed = None

		camera_thread = threading.Thread(target=self.update_camera_feed)
		camera_thread.daemon = True
		camera_thread.start()

	def read_camera(self):
		return deepcopy(self._latest_feed)

	def update_camera_feed(self):
		while True:
			feed = self._camera.read_camera()
			if feed is not None:
				self._latest_feed = self._camera.read_camera()

	def disable_camera(self):
		self._camera.disable_camera()
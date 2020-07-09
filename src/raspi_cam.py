#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import rospy
import time
import cv2

from sensor_msgs.msg import CompressedImage

def main():
	rospy.init_node("raspicam", disable_signals=True)
	rospy.on_shutdown(myhook)
	print("-- Node: raspicam")

	pub = rospy.Publisher('input_image', CompressedImage, queue_size=1)

	size = 320, 240
	with PiCamera() as camera:
		with PiRGBArray(camera, size=size) as stream:
			camera.resolution = size
			camera.rotation = 180
			camera.framerate = 24

			print("-- Initializing...")
			time.sleep(2)
			print(">> Publishing to topic /input_image")

			for frame in camera.capture_continuous(stream, format="bgr", use_video_port=True):
				img = frame.array
				stream.truncate(0)

				bridge = CvBridge()
				try:
					msg = bridge.cv2_to_compressed_imgmsg(img)
					pub.publish(msg)
				except CvBridgeError as e:
					print(e)
					break

				# cv2.imshow("Frame", img)
				# if cv2.waitKey(1) & 0xFF is ord('q'):
					# break

	# cv2.destroyAllWindows()

def myhook():
  print(" shutdown!")



if __name__ == '__main__':
	main()
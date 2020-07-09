#!/usr/bin/env python

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import imutils
import time
import cv2

def blob_detect(image,				# source
				hsv_min,			# (h, s, v)min
				hsv_max,			# (h, s, v)max
				blur=0,				# cv2.blur kernel size
				blob_params=None,	# cv2.SimpleBlobDetector params
				search_window=None,	# Rect [x1, y1, x2, y2]
				imshow=False):		# show process windows
					
	if imshow:
		cv2.imshow("Original", image)
		cv2.waitKey(1)

	if search_window:
		image = apply_search_window(image, search_window)
	
	if blur > 0:
		crop = cv2.blur(image, (blur, blur))

	if imshow:
		cv2.imshow("Blur + crop", crop)
		cv2.waitKey(1)

	
	hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(hsv, hsv_min, hsv_max)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	
	if imshow:
		cv2.imshow("Mask", mask)
		cv2.waitKey(1)

	if not blob_params:
		params = cv2.SimpleBlobDetector_Params()

		params.minThreshold = 0
		params.maxThreshold = 100
		
		params.filterByArea = True
		params.minArea = 30
		params.maxArea = 20000

		params.filterByCircularity = True
		params.minCircularity = 0.1

		params.filterByConvexity = True
		params.minConvexity = 0.5

		params.filterByInertia = True
		params.minInertiaRatio = 0.5
	else:
		params = blob_params

	detector = cv2.SimpleBlobDetector_create(params)

	reversemask = 255 - mask
	keypoints = detector.detect(reversemask)
	return keypoints, reversemask

def get_output_image(image, keypoints):
	for keypoint in keypoints:
		x, y = keypoint.pt
		r = keypoint.size / 2
		if r > 8:
			cv2.circle(image, (int(x), int(y)), int(r), (0, 255, 255), 2)
			cv2.circle(image, (int(x), int(y)), 3, (0, 0, 255), -1)
		break
	return image

def apply_search_window(image, window):
	w, h, ch = image.shape
	a, b, c, d = window
	return image[int(a*w):int(b*w), int(c*h):int(d*h)]
	
def get_blob_relativepos(mask, keypoint):
	cols, rows = mask.shape
	center = 0.5*cols, 0.5*rows		# get center pos

	# get pos relative to center
	x = (keypoint.pt[0] - center[0]) / center[0]
	y = (keypoint.pt[1] - center[1]) / center[1]
	return x, y


def main():
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.rotation = 180
	camera.framerate = 25
	rawCapture = PiRGBArray(camera, size=(640, 480))

	lower = (0, 25, 35)
	upper = (5, 255, 255)

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array

		window = [0.2, 0.8, 0.0, 1.0]

		pts, mask = blob_detect(image, lower, upper, blur=5, imshow=False)

		# for p in pts:
			# print(p.size)
			# print(p.pt)

		output = get_output_image(image, pts)
		cv2.imshow("Output", output)

		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

		key = cv2.waitKey(2) & 0xFF
		if key == ord("q"):
			break


	

if __name__ == '__main__':
	main()
		
	
	

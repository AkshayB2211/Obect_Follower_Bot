#!/usr/bin/env python

import sys
import rospy
import cv2
import time

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError

from blob_detector import *

class BlobDetector:
	def __init__(self, thr_min, thr_max, blur=0, blob_params=None, search_window=None, imshow=False):
		self.thr_min = thr_min
		self.thr_max = thr_max
		self.blur = blur
		self.blob_params = blob_params
		self.search_window = search_window
		self.imshow = imshow
		
		print(">> Publishing image to topic /image_blob")
		print(">> Publishing mask to topic /image_mask")
		print(">> Publishing pos to topic /point_blob")
		self.image_pub = rospy.Publisher("/image_blob", CompressedImage, queue_size=1)
		self.mask_pub = rospy.Publisher("/image_mask", CompressedImage, queue_size=1)
		self.blob_pub = rospy.Publisher("/point_blob", Point32, queue_size=1)
		
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/input_image", CompressedImage, self.callback)
		print("<< Subscribed to topic /input_image")


	def callback(self, data):
		try:
			cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
			rows, cols, chnls = cv_image.shape			

			keypoints, mask = blob_detect(cv_image, self.thr_min, self.thr_max, self.blur,
										  blob_params=self.blob_params,
										  search_window=self.search_window,
										  imshow=self.imshow)

			output = get_output_image(cv_image, keypoints)

			if self.imshow:
				cv2.imshow("Output image", output)
				cv2.waitKey(1)

			self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(output))
			self.mask_pub.publish(self.bridge.cv2_to_compressed_imgmsg(mask))

			# keypoints are 2d objects with pt variable
			for keypoint in keypoints:
				x, y = get_blob_relativepos(mask, keypoint)

				msg = Point32(x, y, 0)
				self.blob_pub.publish(msg)
				break
				
		except CvBridgeError as e:
			print(e)

		

def main():
	lower = (0, 25, 35)
	upper = (5, 255, 255)
	blur = 5

	window = [0.2, 0.8, 0.4, 0.9]

	print(' Node: blob_detector')
	rospy.init_node('blob_detector', anonymous=True)
	bd = BlobDetector(lower, upper, blur=blur, imshow=True)
	
	rospy.spin()
		
	print(' Shutting down')
	cv2.destroyAllWindows()
			
if __name__ == '__main__':
	main()

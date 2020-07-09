#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2

from sensor_msgs.msg import CompressedImage

def main():
	rospy.init_node("raspicam", disable_signals=True)
	rospy.on_shutdown(myhook)
	print("-- Node: raspicam")
	
	pub = rospy.Publisher('input_image', CompressedImage, queue_size=1)

	cam = cv2.VideoCapture(0)
	cam.set(3, 320)			# width
	cam.set(4, 240)			# height
	cam.set(5, 24)			# fps
	if(not cam.isOpened()):
		return

	print(">> Publishing image to topic /input_image")
	while not rospy.is_shutdown():
		ret, img = cam.read()
		if ret:
			img = cv2.flip(img, -1)

			bridge = CvBridge()
			try:
				msg = bridge.cv2_to_compressed_imgmsg(img)
				pub.publish(msg)
			except CvBridgeError as e:
				print(e)
				break

			cv2.imshow("Frame", img)
			if cv2.waitKey(1) & 0xFF is ord('q'):
				cam.release()					
				cv2.destroyAllWindows()
				break

def myhook():
  print(" shutdown!")

		

if __name__ == '__main__':
	main()

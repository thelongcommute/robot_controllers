#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class pid_controller:

  def __init__(self):
	self.image_pub = rospy.Publisher("/R1/pi_camera/image_circle",Image)
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
	self.clock_sub = rospy.Subscriber("/clock", Clock)
	self.move_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
	self.license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)
	self.last_proportional = 0
	self.last_middle = 0
	self.state = 0
	self.counter = 0
	self.time = 0
	self.init_time = 0
	# self.clock = rospy.Subscriber("/clock", String, queue_size=1)

	# state 0: wait 10s for everything to load
	# state 1: turn for 3s to move next to the outer line
	# state 2: PID control off the outerline
	# state 3: Red line detected. Wait for pedestrian
	# state 4: Move robot straight until red line detected again: state -> 2


  def callback(self,data):
	# wait until the world loads

	# For time trial: send a terminal message after 70s (approximately one full lap)
	if rospy.get_time() - self.init_time > 70:
		self.license_plate_pub.publish(str("TeamRed,multi21,-1,XXXX"))



	if self.state == 0:
	  self.time = rospy.get_time()
	  print("waiting: ", self.time)
	  if self.time > 10:
		# start time
		self.license_plate_pub.publish(str("TeamRed,multi21,0,XXXX"))
		self.state = 1
		print("start: ", self.time)

	# move next to the outer line

	if self.state == 1:
	  time = rospy.get_time() - self.time
	  print("moving: ", time)
	  if time < 3:
		move = Twist()
		move.angular.z = 0.5
		move.linear.x = 0.14
		self.move_pub.publish(move)

	  else:
	  	self.time = rospy.get_time()
	  	self.init_time = self.time

		self.state = 2

	  
	# PID Control

	if self.state == 2:
	  try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  except CvBridgeError as e:
		print(e)

	  run_time = rospy.get_time() - self.time

	  if run_time > 0.5:
	  	# Check for red line
	  	redLine = cv_image[680, 640]
	  	if redLine[0] < 100 and redLine[1] < 100 and redLine[2] > 200:
			print("Red Line Detected!")
			self.state = 3

	  # out = cv2.VideoWriter('outpy.mp4',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
	  gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	  ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
	  startPixel = -1
	  endPixel = -1
   
	  for i in range(640, 1280):
		  val = thresh[650,i]   # val is 0 when it sees black, 255 when it sees white
		  
		  if val == 255:    # if you see black
			  if startPixel == -1:
				  startPixel = i
	  
		  else:           # if you see white
			  if endPixel == -1 and startPixel != -1:
				  endPixel = i
				  break

	  middle = startPixel + (endPixel - startPixel)//2

	  if middle <= startPixel + 10:
		middle = self.last_middle

	  # stay 480 px away from the middle of the outer line
	  middle -= 480

	  view_image = cv_image
	  cv2.circle(view_image, (middle,650), 15, (0,0,255), -1) # y,x coordinate

	  proportional = abs(middle-640)
	  derivative = abs(proportional-self.last_proportional)
	  power_difference = proportional/80 + derivative/90;
	  # print(str(proportional) + "   " + str(self.last_proportional) + "   " + str(power_difference))
	  move = Twist()
	  if middle < 640:
		move.angular.z = 0.15 + float(0.2 * power_difference)
	  if middle > 640:
		move.angular.z = -(0.15 + float(0.2 * power_difference))

	  move.linear.x = 0.15
	  self.move_pub.publish(move)

	  self.last_middle = middle
	  self.last_proportional = proportional

	  cv2.imshow("view image", view_image)
	  cv2.waitKey(3)
	  self.counter += 1
	  try:
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	  except CvBridgeError as e:
		print(e)

	if self.state == 3:

	  # stop

	  move = Twist()
	  move.angular.z = 0
	  move.linear.x = 0
	  self.move_pub.publish(move)

	  try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
	  except CvBridgeError as e:
		print(e)

	  ret, thresh = cv2.threshold(cv_image, 150, 255, cv2.THRESH_BINARY)

	  ped = thresh[430, 650]
	  if ped == 0:
		print("pedestrian Detected!!")
		self.time = rospy.get_time()
		self.state = 4


	  cv2.imshow("view image", thresh)
	  cv2.waitKey(3)

	if self.state == 4:
	  
	  run_time = rospy.get_time() - self.time
	  print(run_time)
	  try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  except CvBridgeError as e:
		print(e)

	  cv2.imshow("view image", cv_image)
	  cv2.waitKey(3)

	  move = Twist()
	  move.angular.z = 0
	  move.linear.x = 0.1	
	  self.move_pub.publish(move)

	  if run_time > 0.5:
		# Check for red line, go back to pid control when is detected.
	  	redLine = cv_image[680, 640]
	  	if redLine[0] < 100 and redLine[1] < 100 and redLine[2] > 200:
	  		self.time = rospy.get_time()
			self.state = 2


def main(args):
  rospy.init_node('pid_controller', anonymous=True)
  pc = pid_controller()
  
  try:
	rospy.spin()
  except KeyboardInterrupt:
	print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
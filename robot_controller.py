#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from perspective_transform_script import process_image_and_flatten
from segment_license_plate_images import segment_image

import tensorflow as tf
from tensorflow import keras

class pid_controller:

  def __init__(self):

	# define subscribers, publishers and variables
	self.image_pub = rospy.Publisher("/R1/pi_camera/image_circle",Image)
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
	self.clock_sub = rospy.Subscriber("/clock", Clock)
	self.move_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
	self.license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)

	# save previous proportional constant and middle value for PID control
	self.last_proportional = 0
	self.last_middle = 0

	# Initial State: 0
	self.state = 0

	# self.time is used to assign the time right before a state transition.
	# Then we can caluclate for how long a state has been running
	# by the line run_time = rospy.get_time() - self.time
	self.time = 0

	# init_time is used to store the time when state 2 (PID) started.
	self.init_time = 0

	# flag after sending the terminal message to the license plate.
	self.flag = 0
	# count how many consecutive times the same plate ID has been read
	self.det_count = 0

	# for state 8: if parked car is detected
	self.isDetected = False
	# check if we are in inner loop
	self.isInner = False
	# check if the model is loaded
	self.isLoaded = False

	# initialize plate detector model as None, later defined in State 1
	self.model = None
	# previous license plate
	self.previous_plate_id = "XXXX"
	# number of published attempts
	self.published_count = 0
	# history of published id
	self.published_license_plates = [[],[],[],[],[],[],[],[]]

	# <State List> 
	# state 0: wait 10s for everything to load
	# state 1: turn for 3s to move next to the outer line
	# state 2: PID control off the outerline of outer ring
	# state 3: Red line detected. Wait for pedestrian
	# state 4: Move robot straight until red line detected again: state -> 2
	# state 5: Left PID for cornering the outer line of the inner ring,
	#          used for initial cornering as well as after vehicle has passed
	# state 6: Stop and wait until the vehicle has been detected
	# state 7: wait for 3 seconds after the vehicle is detected, then state 6 -> 5
	# state 8: Left PID for inner line of inner ring. Look for parked cars
	# state 9: Parked car has been detected, move straight for 3.5s, then state 9 -> 8

  def int_to_char(self, numeric):
    ascii_int_offset = 48
    ascii_alpha_offset = 65
    onehot_int_offset = 26

    if numeric < onehot_int_offset:
        return chr(numeric + ascii_alpha_offset)
    return chr(numeric - onehot_int_offset + ascii_int_offset)

  # return the most common item in list
  def most_common(self, lst):
	return max(set(lst), key=lst.count)

  # Read license plate
  def read_license_plate(self, perp_transformed):

	cv2.imshow("parked car image", perp_transformed)
	cv2.waitKey(3)

	# get numpy array of segmented characters of the plate
	char_imgs = np.array(segment_image(perp_transformed))

	# process data for predict
	char_imgs = char_imgs/255.
	char_imgs = np.expand_dims(char_imgs,axis=3)

	# predict data
	char_predict = self.model.predict(char_imgs)

	plate_location = self.int_to_char(26 + np.argmax(char_predict[0][26:]))
	# index 0 is the plate location
	plate_id_list = [plate_location]

	# read first two alphabets of the license plate
	for i in range(1,5):
		# alphabetical
		if i <= 2:
			char = self.int_to_char(np.argmax(char_predict[i][0:26]))
			plate_id_list.append(char)
		# numerical
		else:
			char = self.int_to_char(26 + np.argmax(char_predict[i][26:]))
			plate_id_list.append(char)

	plate_id_string = "".join(plate_id_list)

	# plate location + plate id
	return plate_id_string

  def callback(self,data):

	# # JUST FOR TIME TRIAL: send a terminal message after 70s (approximately one full lap)
	# if rospy.get_time() - self.init_time > 70 and self.flag == 0:
	# 	self.license_plate_pub.publish(str("Team20,password,-1,XXXX"))
	# 	self.flag = 1
	
	# Wait for model to load, 5 additional seconds and start moving
	if self.state == 0: 
	  # load cnn model
	  if not self.isLoaded:
		self.model = keras.models.load_model("plate_detector_model")
		self.isLoaded = True
		self.time = rospy.get_time()
	  
	  if self.isLoaded:
		print("waiting: ", rospy.get_time()-self.time)
		if rospy.get_time() - self.time > 5:	  		
			# start time
			self.license_plate_pub.publish(str("Team20,password,0,XXXX"))
			self.state = 1
			self.time = rospy.get_time()
			print("start: ", self.time)

	# move next to the outer line
	if self.state == 1:
	  move = Twist()
		
	  time = rospy.get_time() - self.time
	  print("moving: ", time)

	  if time < 3:
		move.angular.z = 0.1
		move.linear.x = 0.1
		self.move_pub.publish(move)

	  elif time < 5:
		move.angular.z = 0.6
		move.linear.x = 0.05
		self.move_pub.publish(move)

	  else:
		self.time = rospy.get_time()
		self.init_time = self.time
		self.state = 2

	# State 2: Right lane PID Control in outer ring
	if self.state == 2:
	  try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  except CvBridgeError as e:
		print(e)

	 #  # NEEDS TO BE UPDATED: ONLY FOR DEBUGGING
	 #  # activate left pid after 14 seconds
	 #  if rospy.get_time() - self.init_time >= 14:
		# print("left pid activate")
		# self.time = rospy.get_time()
		# self.state = 5

	  self.isDetected = False

	  view_image = cv_image

	  blue_box_left = cv_image[375:400, 400:500]
	  # find average color of box
	  avg_color_per_row_left = np.average(blue_box_left, axis=0)
	  avg_color_left = np.average(avg_color_per_row_left, axis=0)

	  if 95 <= avg_color_left[0] <= 130 and avg_color_left[1] <= 30 and avg_color_left[2] <= 30:

		self.isDetected = True

		perp_transformed = process_image_and_flatten(cv_image, car_side=0)
		if not perp_transformed is None:
			# read license plate
			plate_id = self.read_license_plate(perp_transformed)
			print("current plate id: {}".format(plate_id))

			# store plate ID and plate location
			plate_loc = int(plate_id[0])
			plate_id_str = plate_id[1:]

			# append plate id to the corresponding location in list. ex) P1 -> index 0
			self.published_license_plates[plate_loc - 1].append(plate_id_str)

			# # if current reading is equal to the last, increment det_count
			# if plate_id == self.previous_plate_id:
			# 	self.det_count += 1
			# # if same reading for more than 3 times, it is the correct ID:
			# if self.det_count >= 4:
			# 	print("FINAL plate id: {}  plate location: {}".format(plate_id[1:], plate_id[0]))
			# 	self.det_count = 0

			# get the most frequent plate ID in the corresponding location
			plate_id_most_common = self.most_common(self.published_license_plates[plate_loc-1])

			# # publish to license plate topic iff hasn't been published before
			# if plate_id[0] not in self.published_license_plates:
			self.license_plate_pub.publish(str("Team20,password,{},{}".format(plate_loc, plate_id_most_common)))

			# if plate_id[0] not in self.published_license_plates:
			# 	self.published_license_plates.append(plate_id[0])

			published_count = len(self.published_license_plates)

			print("published count: ", published_count)
			print("published_license_plates at location {}: ".format(plate_loc), self.published_license_plates[plate_loc-1])

			# plate location 1,3,4,6 are valid to start driving to Inner Loop
			if plate_loc == 1 and len(self.published_license_plates[0]) >= 12:
				print("driving straight for 3.5s")
				self.time = rospy.get_time()
				self.state = 11

			self.previous_plate_id = plate_id

	  elif 195 <= avg_color_left[0] <= 225 and 95 <= avg_color_left[1] <= 125 and 95 <= avg_color_left[2] <= 125:

		self.isDetected = True

		perp_transformed = process_image_and_flatten(cv_image, car_side=0)
		if not perp_transformed is None:
			# read license plate
			plate_id = self.read_license_plate(perp_transformed)
			print("current plate id: {}".format(plate_id))

			# store plate ID and plate location
			plate_loc = int(plate_id[0])
			plate_id_str = plate_id[1:]

			# append plate id to the corresponding location in list. ex) P1 -> index 0
			self.published_license_plates[plate_loc - 1].append(plate_id_str)

			# # if current reading is equal to the last, increment det_count
			# if plate_id == self.previous_plate_id:
			# 	self.det_count += 1
			# # if same reading for more than 3 times, it is the correct ID:
			# if self.det_count >= 4:
			# 	print("FINAL plate id: {}  plate location: {}".format(plate_id[1:], plate_id[0]))
			# 	self.det_count = 0

			# get the most frequent plate ID in the corresponding location
			plate_id_most_common = self.most_common(self.published_license_plates[plate_loc-1])

			# # publish to license plate topic iff hasn't been published before
			# if plate_id[0] not in self.published_license_plates:
			self.license_plate_pub.publish(str("Team20,password,{},{}".format(plate_loc, plate_id_most_common)))

			# if plate_id[0] not in self.published_license_plates:
			# 	self.published_license_plates.append(plate_id[0])

			published_count = len(self.published_license_plates)

			print("published count: ", published_count)
			print("published_license_plates at location {}: ".format(plate_loc), self.published_license_plates[plate_loc-1])

			# plate location 1,3,4,6 are valid to start driving to Inner Loop
			if plate_loc == 1 and len(self.published_license_plates[0]) >= 12:
				print("driving straight for 3.5s")
				self.time = rospy.get_time()
				self.state = 11

			self.previous_plate_id = plate_id

	  # find run time of state 2
	  run_time = rospy.get_time() - self.time
	  if run_time > 0.5:
		# Check for red line
		redLine = cv_image[680, 640]
		if redLine[0] < 100 and redLine[1] < 100 and redLine[2] > 200:
			print("Red Line Detected!")
			self.state = 3

	  # grayscale and threshold the image 
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

	  self.last_middle = middle
	  # stay 480 px away from the middle of the outer line
	  middle -= 480

	  # if middle is less than 570, it is a corner. When cornering,
	  # we should stay 200px more away from the outerline.
	  if middle < 570:
		middle -= 200

	  cv2.circle(view_image, (middle,650), 15, (0,0,255), -1) # y,x coordinate

	  proportional = abs(middle-640)
	  derivative = abs(proportional-self.last_proportional)
	  power_difference = proportional/80 + derivative/90;
	  move = Twist()
	  move.linear.x = 0.2

	  if self.isDetected:
		move.linear.x = 0.09

	  if middle < 640:
		move.angular.z = 0.15 + float(0.2 * power_difference)
		if middle < 400:
			move.linear.x = 0
		
	  if middle > 640:
		move.angular.z = -(0.15 + float(0.2 * power_difference))
		if middle > 880:
			move.linear.x = 0
		

	  self.move_pub.publish(move)
	  self.last_proportional = proportional

	  # cv2.imshow("view image", view_image)
	  # cv2.waitKey(3)
	  try:
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	  except CvBridgeError as e:
		print(e)

	# Red line detected. Stop and detect for pedestrian
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

	  # cv2.imshow("view image", thresh)
	  # cv2.waitKey(3)

	# State 4: Drive straight until red line is detected
	if self.state == 4:
	  
	  run_time = rospy.get_time() - self.time

	  try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  except CvBridgeError as e:
		print(e)

	  # cv2.imshow("view image", cv_image)
	  # cv2.waitKey(3)

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

	# State 5: Left PID Cornering
	if self.state == 5:
		print("Cornering: ", rospy.get_time() - self.time)
		# isInner = False when we are first cornering into inner loop. state 5 -> 6
		# isInner = True after the vehicle passes, PID off the inner ring's outerline for 3 seconds then
		# 			start PID off the inner ring's inner line. state 5 -> 8
		if self.isInner:
			if rospy.get_time() - self.time >= 3.5:
				print("start PID control on the inner most line")
				self.state = 8
		else:
			if rospy.get_time() - self.time >= 6.6:
				print("stop and wait for vehicle")
				self.state = 6

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		view_image = cv_image

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
		startPixel = -1
		endPixel = -1

		for i in range(640):
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

		cv2.circle(view_image, (middle,650), 15, (0,255,0), -1)
		self.last_middle = middle

		# stay 480 px away from the middle of the inner line
		middle += 480

		# if middle is less than 570, it is a corner. When cornering,
		# we should stay 200px more away from the outerline.
		if middle < 570:
			middle -= 200

		# cv2.circle(view_image, (middle,650), 15, (0,0,255), -1) # y,x coordinate
		# cv2.imshow("view image", view_image)
		# cv2.waitKey(3)

		proportional = abs(middle-640)
		derivative = abs(proportional-self.last_proportional)
		power_difference = proportional/80 + derivative/90;
		# print(str(proportional) + "   " + str(self.last_proportional) + "   " + str(power_difference))
		move = Twist()
		move.linear.x = 0.15

		if middle < 640:
			move.angular.z = 0.15 + float(0.2 * power_difference)
		if middle < 400:
			move.linear.x = 0

		if middle > 640:
			move.angular.z = -(0.15 + float(0.2 * power_difference))
		if middle > 880:
			move.linear.x = 0

		self.move_pub.publish(move)
		self.last_proportional = proportional

	# State 6: Detect the vehicle
	if self.state == 6:

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		# stop robot
		move = Twist()
		move.angular.z = 0
		move.linear.x = 0
		self.move_pub.publish(move) 

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

		# find the color or vehicle detect box
		vehicle_detect_box = thresh[440:450, 640:680]
		avg_color_per_row_vehicle = np.average(vehicle_detect_box, axis=0)
		avg_color_vehicle = np.average(avg_color_per_row_vehicle, axis=0)
	  
		print(avg_color_vehicle)
		if avg_color_vehicle > 12:
			print("vehicle detected")
			self.time = rospy.get_time()
			self.state = 7

		cv2.imshow("view image", thresh)
		cv2.waitKey(3)

	# State 7: Wait for 5 seconds until the vehicle passes completely
	if self.state == 7:
		print("waiting: ", rospy.get_time() - self.time)
		if rospy.get_time() - self.time > 3:
			print("you can go now")
			self.isInner = True
			self.time = rospy.get_time()
			self.state = 5

	# State 8: Inner loop Left PID
	if self.state == 8:
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		# stop robot 

		view_image = cv_image

		blue_box_right = cv_image[380:400, 780:840]
		# find average color of box
		avg_color_per_row_right = np.average(blue_box_right, axis=0)
		avg_color_right = np.average(avg_color_per_row_right, axis=0)

		if 95 <= avg_color_right[0] <= 130 and avg_color_right[1] <= 30 and avg_color_right[2] <= 30:

			self.isDetected = True
			self.time = rospy.get_time()

			perp_transformed = process_image_and_flatten(cv_image, car_side=0)
			if not perp_transformed is None:

				# read license plate
				plate_id = self.read_license_plate(perp_transformed)
				print("current plate id: {}".format(plate_id))

				# store plate ID and plate location
				plate_loc = int(plate_id[0])
				plate_id_str = plate_id[1:]

				# append plate id to the corresponding location in list. ex) P1 -> index 0
				self.published_license_plates[plate_loc - 1].append(plate_id_str)

				# # if current reading is equal to the last, increment det_count
				# if plate_id == self.previous_plate_id:
				# 	self.det_count += 1
				# # if same reading for more than 3 times, it is the correct ID:
				# if self.det_count >= 4:
				# 	print("FINAL plate id: {}  plate location: {}".format(plate_id[1:], plate_id[0]))
				# 	self.det_count = 0

				# get the most frequent plate ID in the corresponding location
				plate_id_most_common = self.most_common(self.published_license_plates[plate_loc-1])

				# # publish to license plate topic iff hasn't been published before
				# if plate_id[0] not in self.published_license_plates:
				self.license_plate_pub.publish(str("Team20,password,{},{}".format(plate_loc, plate_id_most_common)))

				# if plate_id[0] not in self.published_license_plates:
				# 	self.published_license_plates.append(plate_id[0])

				published_count = len(self.published_license_plates)

				print("published count: ", published_count)
				print("published_license_plates at location {}: ".format(plate_loc), self.published_license_plates[plate_loc-1])

				# send termination message after we publish all license plates!!!
				if len(self.published_license_plates[6]) >= 5 and len(self.published_license_plates[7]) >= 5:
					self.license_plate_pub.publish(str("Team20,password,-1,XXXX"))
					self.state = 10
					print("-----Run Complete-----")

				self.previous_plate_id = plate_id

		elif 195 <= avg_color_right[0] <= 225 and 95 <= avg_color_right[1] <= 125 and 95 <= avg_color_right[2] <= 125:

			self.isDetected = True
			self.time = rospy.get_time()

			perp_transformed = process_image_and_flatten(cv_image, car_side=0)
			if not perp_transformed is None:

				# read license plate
				plate_id = self.read_license_plate(perp_transformed)
				print("current plate id: {}".format(plate_id))

				# store plate ID and plate location
				plate_loc = int(plate_id[0])
				plate_id_str = plate_id[1:]

				# append plate id to the corresponding location in list. ex) P1 -> index 0
				self.published_license_plates[plate_loc - 1].append(plate_id_str)

				# # if current reading is equal to the last, increment det_count
				# if plate_id == self.previous_plate_id:
				# 	self.det_count += 1
				# # if same reading for more than 3 times, it is the correct ID:
				# if self.det_count >= 4:
				# 	print("FINAL plate id: {}  plate location: {}".format(plate_id[1:], plate_id[0]))
				# 	self.det_count = 0

				# get the most frequent plate ID in the corresponding location
				plate_id_most_common = self.most_common(self.published_license_plates[plate_loc-1])

				# # publish to license plate topic iff hasn't been published before
				# if plate_id[0] not in self.published_license_plates:
				self.license_plate_pub.publish(str("Team20,password,{},{}".format(plate_loc, plate_id_most_common)))

				# if plate_id[0] not in self.published_license_plates:
				# 	self.published_license_plates.append(plate_id[0])

				published_count = len(self.published_license_plates)

				print("published count: ", published_count)
				print("published_license_plates at location {}: ".format(plate_loc), self.published_license_plates[plate_loc-1])

				# send termination message after we publish all license plates!!!
				if len(self.published_license_plates[6]) >= 5 and len(self.published_license_plates[7]) >= 5:
					self.license_plate_pub.publish(str("Team20,password,-1,XXXX"))
					self.state = 10
					print("-----Run Complete-----")


				self.previous_plate_id = plate_id

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

		cv2.circle(view_image, (middle,650), 15, (0,255,0), -1)
		self.last_middle = middle
		# stay 480 px away from the middle of the outer line
		middle -= 240

		cv2.circle(view_image, (middle,650), 15, (0,0,255), -1) # y,x coordinate

		proportional = abs(middle-640)
		derivative = abs(proportional-self.last_proportional)
		power_difference = proportional/80 + derivative/90;
		# print(str(proportional) + "   " + str(self.last_proportional) + "   " + str(power_difference))
		move = Twist()
		move.linear.x = 0.12

		if middle < 640:
			move.angular.z = 0.15 + float(0.2 * power_difference)
		if middle < 400:
			move.linear.x = 0

		if middle > 640:
			move.angular.z = -(0.15 + float(0.2 * power_difference))
		if middle > 880:
			move.linear.x = 0

		# if parked car is detected, go straight for 3.5s.
		# this if statement replace state 9
		if self.isDetected:
			move.angular.z = 0
			move.linear.x = 0.06

			if rospy.get_time() - self.time > 3.5:
				self.isDetected = False

		self.move_pub.publish(move)
		self.last_proportional = proportional

		# cv2.imshow("view image", view_image)
		# cv2.waitKey(3)

	# # State 9: drive straight until we pass the parked vehicle
	# if self.state == 9:
	# 	try:
	# 		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	# 	except CvBridgeError as e:
	# 		print(e)

	# 	cv2.imshow("view image", cv_image)
	# 	cv2.waitKey(3)

	# 	move = Twist()
	# 	move.angular.z = 0
	# 	move.linear.x = 0.1   
	# 	self.move_pub.publish(move)

	# 	# Go back to State 8: Inner Loop PID
	# 	if rospy.get_time() - self.time > 3.5:
	# 		self.state = 8


	# finished competition
	if self.state == 10:
		move = Twist()
		move.angular.z = 0
		move.linear.x = 0  
		self.move_pub.publish(move)

	if self.state == 11:
		move = Twist()
		move.angular.z = 0
		move.linear.x = 0.1   
		self.move_pub.publish(move)
		print(rospy.get_time() - self.time)

		# Go back to State 8: Inner Loop PID
		if rospy.get_time() - self.time > 4:
			self.state = 5
			self.time = rospy.get_time()
			print("left PID activated")

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
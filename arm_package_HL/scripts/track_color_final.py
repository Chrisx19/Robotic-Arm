#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32 # Messages used in the node must be imported.
from geometry_msgs.msg import Pose

import cv2
import numpy as np
import time

rospy.init_node("track_blob")

prev_frame_time = 0
new_frame_time = 0

cap = cv2.VideoCapture(0)

pub = rospy.Publisher('follow_blob', Pose, queue_size=10)

target_pose = Pose() # declaring a message variable of type Int32


x_d=0.0
y_d=0.0
x_d_p=0.0
y_d_p=0.0

while(1):
	_, webcam = cap.read()
	    
	#converting frame(webcam i.e BGR) to HSV (hue-saturation-value)

	hsv=cv2.cvtColor(webcam,cv2.COLOR_BGR2HSV)

	blue_lower=np.array([94, 80, 2],np.uint8)
	blue_upper=np.array([120,255,255],np.uint8)
	blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")
	blue_mask = cv2.dilate(blue_mask, kernal)
	res_blue = cv2.bitwise_and(webcam, webcam, mask = blue_mask)

	webcam=cv2.circle(webcam, (260,68), 5, (255,0,0), -1)

	font = cv2.FONT_HERSHEY_SIMPLEX
	new_frame_time = time.time()
	fps = 1/(new_frame_time-prev_frame_time)
	prev_frame_time = new_frame_time
	fps = int(fps)
	fps = str(fps)

	cv2.putText(webcam, fps, (19, 25), font, 0.8, (100, 255, 0), 1, cv2.LINE_AA)
	
			
	#Tracking the Blue Color
	contours,hierarchy = cv2.findContours(blue_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	#Need to add Red and Green for detection
	if len(contours)>0:
		contour= max(contours,key=cv2.contourArea)
		area = cv2.contourArea(contour)
		if area>300: 
			x,y,w,h = cv2.boundingRect(contour)	
			webcam = cv2.rectangle(webcam, (x,y), (x+w,y+h), (255,0,0), 2)
			
			circle_center_x = int((2*x+w)/2)                                                #center needs to be integral
			circle_center_y = int((2*y+h)/2)
			webcam=cv2.circle(webcam, (circle_center_x, circle_center_y), 5, (255,0,0), -1)	#dot to the object being detected
			
			line_center_x  = int((2*x+w)/2)
			line_center_y = int((2*y+h)/2)
			webcam=cv2.line(webcam, (260,68), (line_center_x, line_center_y), (0,255,0), 2)	#creates the line between reference and block
		
			x_d= (((2*y+h)/2)-68) * 0.06													#calculation distance
			y_d= (((2*x+w)/2)-260) * 0.075
			
			s = 'x_d:'+ str(x_d) + '   y_d:'+str(y_d)				#output coordinates
			
			cv2.putText(webcam,s,(x-20,y-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1,cv2.LINE_AA)

			#ros stuff, (chris) got this below
			if (abs(x_d-x_d_p)> 1 or abs(y_d-y_d_p)>1):								#calculation for driving to grab the block
				target_pose.position.x=x_d*0.01
				target_pose.position.y=y_d*0.01
				target_pose.position.z=0.0

				pub.publish(target_pose)											#ros publish
			
				x_d_p=x_d
				y_d_p=y_d
			
	cv2.imshow("Mask",blue_mask)
	cv2.imshow("Color Tracking",webcam)
	if cv2.waitKey(1)== ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

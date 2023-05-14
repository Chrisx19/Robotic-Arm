import cv2
import numpy as np
import time
import math
from realsense_depth import *

prev_frame_time = 0
new_frame_time = 0

# cap = cv2.VideoCapture(4)
cap = DepthCamera()

x_d=0.0
y_d=0.0
x_d_p=0.0
y_d_p=0.0

point = (400, 300)

#call for depth detection
def show_distance(x, y):
    global point
    point = (x, y)

# Create mouse event
cv2.namedWindow("Color Tracking")

while(1):
    # _, webcam = cap.read()
    ret, depth_frame, webcam = cap.get_frame()
    hsv=cv2.cvtColor(webcam,cv2.COLOR_BGR2HSV)

    distance = depth_frame[point[1], point[0]]

    blue_lower=np.array([56, 81, 105],np.uint8)
    blue_upper=np.array([255,255,255],np.uint8)
    blue_mask=cv2.inRange(hsv,blue_lower,blue_upper)

    green_lower = np.array([42, 0, 57], np.uint8)
    green_upper = np.array([100, 146, 134], np.uint8)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    red_lower = np.array([150, 147, 104], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)

    kernal = np.ones((5 ,5), "uint8")

    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(webcam, webcam, mask = blue_mask)

    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(webcam, webcam, mask = green_mask)

    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(webcam, webcam, mask = red_mask)

    font = cv2.FONT_HERSHEY_SIMPLEX
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    fps = str(fps)

    cv2.putText(webcam, fps, (19, 25), font, 0.8, (100, 255, 0), 1, cv2.LINE_AA)

    def findGreen(green_mask, webcam, x_d, y_d, x_d_p, y_d_p):
        webcam=cv2.circle(webcam, (260,68), 5, (255,0,0), -1)
        contours,hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            if len(contours)>0:
                contour= max(contours,key=cv2.contourArea)
                area = cv2.contourArea(contour)
                if area>500: 
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    cv2.drawContours(webcam,[box],0,(0,255,0),2)

                    ######## orientation detection #######
                    webcam=cv2.circle(webcam, (box[0][0], box[0][1]), 5, (0,0,0), -1)
                    webcam=cv2.circle(webcam, (box[1][0], box[1][1]), 5, (255,255,255), -1)
                    webcam=cv2.circle(webcam, (box[2][0], box[2][1]), 5, (0,255,0), -1)
                    webcam=cv2.circle(webcam, (box[3][0], box[3][1]), 5, (0,0,255), -1)
                    cv2.line(webcam, (int(box[2][0]), int(box[2][1])), (int(box[1][0]), int(box[1][1])), (255,255,255),2)
                    cv2.line(webcam, (int(box[2][0]), int(box[2][1])), (800, int(box[2][1])), (255,255,255),2)

                    angle = (math.atan2(-(int(box[1][1]) - int(box[2][1])),(int(box[1][0])-int(box[2][0]))))*(180/math.pi) #angle calculation
                    #######################################

                    cv2.putText(webcam, 
                                'Angle:'+str(angle),
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+40), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))

                    cv2.putText(webcam, "Green Color", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                1.0, (0, 255, 0))
                    
                    circle_center_x = int((2*x+w)/2)                                                #center needs to be integral
                    circle_center_y = int((2*y+h)/2)
                    webcam=cv2.circle(webcam, (circle_center_x, circle_center_y), 5, (255,0,0), -1)	#dot to the object being detected
                    
                    webcam=cv2.circle(webcam, (contour[0][0][0], contour[0][0][1]), 5, (255,255,255), -1)

                    line_center_x  = int((2*x+w)/2)
                    line_center_y = int((2*y+h)/2)
                    webcam=cv2.line(webcam, (260,68), (line_center_x, line_center_y), (0,255,0), 2)	#creates the line between reference and block
                
                    x_d= (((2*y+h)/2)-68) * 0.06							#calculation distance
                    y_d= (((2*x+w)/2)-260) * 0.075

                    if ((circle_center_x > -1) and (circle_center_y > -1)):
                        show_distance(circle_center_x, circle_center_y)
                        cv2.putText(webcam, 
                                    "{}mm".format(distance),
                                    (int(((2*x+w)/2)-20), int(((2*y+h)/2))+50), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.35, (255, 255, 255))

                    cv2.putText(webcam, 
                                'x_d:'+ str(x_d),   #output x_d coordinate
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+20), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))
                    cv2.putText(webcam, 
                                'y_d:'+str(y_d),    #output y_d coordinate
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+30), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))


    def findBlue(blue_mask, webcam, x_d, y_d, x_d_p, y_d_p):
        webcam=cv2.circle(webcam, (260,68), 5, (255,0,0), -1)
        contours,hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            if len(contours)>0:
                contour= max(contours,key=cv2.contourArea)
                area = cv2.contourArea(contour)
                if area>500: 
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    cv2.drawContours(webcam,[box],0,(255,0,0),2)

                    webcam=cv2.circle(webcam, (box[0][0], box[0][1]), 5, (0,0,0), -1)
                    webcam=cv2.circle(webcam, (box[1][0], box[1][1]), 5, (255,255,255), -1)
                    webcam=cv2.circle(webcam, (box[2][0], box[2][1]), 5, (0,255,0), -1)
                    webcam=cv2.circle(webcam, (box[3][0], box[3][1]), 5, (0,0,255), -1)
                    cv2.line(webcam, (int(box[2][0]), int(box[2][1])), (int(box[1][0]), int(box[1][1])), (255,255,255),2)
                    cv2.line(webcam, (int(box[2][0]), int(box[2][1])), (800, int(box[2][1])), (255,255,255),2)

                    angle = (math.atan2(-(int(box[1][1]) - int(box[2][1])),(int(box[1][0])-int(box[2][0]))))*(180/math.pi)

                    cv2.putText(webcam, 
                                'Angle:'+str(angle),
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+40), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))
                    
                    cv2.putText(webcam, "Blue Color", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                1.0, (255, 0, 0))
                    
                    circle_center_x = int((2*x+w)/2)                                                #center needs to be integral
                    circle_center_y = int((2*y+h)/2)
                    webcam=cv2.circle(webcam, (circle_center_x, circle_center_y), 5, (255,0,0), -1)	#dot to the object being detected
                    
                    line_center_x  = int((2*x+w)/2)
                    line_center_y = int((2*y+h)/2)
                    webcam=cv2.line(webcam, (260,68), (line_center_x, line_center_y), (0,255,0), 2)	#creates the line between reference and block
                
                    x_d= (((2*y+h)/2)-68) * 0.06							#calculation distance
                    y_d= (((2*x+w)/2)-260) * 0.075

                    if ((circle_center_x > -1) and (circle_center_y > -1)):
                        show_distance(circle_center_x, circle_center_y)
                        cv2.putText(webcam, 
                                    "{}mm".format(distance),
                                    (int(((2*x+w)/2)-20), int(((2*y+h)/2))+50), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.35, (255, 255, 255))

                    cv2.putText(webcam, 
                                'x_d:'+ str(x_d),   #output x_d coordinate
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+20), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))
                    cv2.putText(webcam, 
                                'y_d:'+str(y_d),    #output y_d coordinate
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+30), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))

    def findRed(red_mask, webcam, x_d, y_d, x_d_p, y_d_p):
        webcam=cv2.circle(webcam, (260,68), 5, (255,0,0), -1)
        contours,hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            if len(contours)>0:
                contour= max(contours,key=cv2.contourArea)
                area = cv2.contourArea(contour)
                if area>500: 
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    cv2.drawContours(webcam,[box],0,(0,0,255),2)

                    webcam=cv2.circle(webcam, (box[0][0], box[0][1]), 5, (0,0,0), -1)
                    webcam=cv2.circle(webcam, (box[1][0], box[1][1]), 5, (255,255,255), -1)
                    webcam=cv2.circle(webcam, (box[2][0], box[2][1]), 5, (0,255,0), -1)
                    webcam=cv2.circle(webcam, (box[3][0], box[3][1]), 5, (0,0,255), -1)
                    cv2.line(webcam, (int(box[2][0]), int(box[2][1])), (int(box[1][0]), int(box[1][1])), (255,255,255),2)
                    cv2.line(webcam, (int(box[2][0]), int(box[2][1])), (800, int(box[2][1])), (255,255,255),2)

                    angle = (math.atan2(-(int(box[1][1]) - int(box[2][1])),(int(box[1][0])-int(box[2][0]))))*(180/math.pi)

                    cv2.putText(webcam, 
                                'Angle:'+str(angle),
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+40), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))
                    
                    cv2.putText(webcam, "Red Color", (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                1.0, (0, 0, 255))
                    
                    circle_center_x = int((2*x+w)/2)                                                #center needs to be integral
                    circle_center_y = int((2*y+h)/2)
                    webcam=cv2.circle(webcam, (circle_center_x, circle_center_y), 5, (255,0,0), -1)	#dot to the object being detected
                    
                    line_center_x  = int((2*x+w)/2)
                    line_center_y = int((2*y+h)/2)
                    webcam=cv2.line(webcam, (260,68), (line_center_x, line_center_y), (0,255,0), 2)	#creates the line between reference and block
                
                    x_d= (((2*y+h)/2)-68) * 0.06							#calculation distance
                    y_d= (((2*x+w)/2)-260) * 0.075

                    if ((circle_center_x > -1) and (circle_center_y > -1)):
                        show_distance(circle_center_x, circle_center_y)
                        cv2.putText(webcam, 
                                    "{}mm".format(distance),
                                    (int(((2*x+w)/2)-20), int(((2*y+h)/2))+50), cv2.FONT_HERSHEY_SIMPLEX, 
                                    0.35, (255, 255, 255))

                    cv2.putText(webcam, 
                                'x_d:'+ str(x_d),   #output x_d coordinate
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+20), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))
                    cv2.putText(webcam, 
                                'y_d:'+str(y_d),    #output y_d coordinate
                                (int(((2*x+w)/2)-20), int(((2*y+h)/2))+30), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.35, (255, 255, 255))

    #findGreen(green_mask, webcam, x_d, y_d, x_d_p, y_d_p)
    findBlue(blue_mask, webcam, x_d, y_d, x_d_p, y_d_p)
    #findRed(red_mask, webcam, x_d, y_d, x_d_p, y_d_p)

    cv2.imshow("depth frame", depth_frame)
    #cv2.imshow("Mask",blue_mask)
    # cv2.imshow("Mask",blue_mask)
    cv2.imshow("Color Tracking",webcam)
    if cv2.waitKey(1)== ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
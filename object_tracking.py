#!/usr/bin/env python3
import numpy as np
import cv2

webcam = cv2.VideoCapture(0)            # Capturing video through webcam

# Start a while loop
while(1):
    
    # Reading the video from the
    # webcam in image frames
    _, imageFrame = webcam.read()
  
    # Convert the imageFrame in 
    # BGR(RGB color space) to 
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
  
    # Set range for red color and 
    # define mask
    red_lower = np.array([150, 147, 104], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
  
    # Set range for green color and 
    # define mask
    green_lower = np.array([44, 86, 72], np.uint8)
    green_upper = np.array([141, 209, 206], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
  
    # Set range for blue color and
    # define mask
    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
      
    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")
      
    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                              mask = red_mask)
      
    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask = green_mask)
      
    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                               mask = blue_mask)
   
    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 500):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h), 
                                       (0, 0, 255), 2)
            imageFrame = cv2.circle(imageFrame,
                                    (int((2*x+w)/2), int((2*y+h)/2)),
                                    5,(255,255,255), cv2.FILLED)

            cv2.putText(imageFrame, 
                        "X: " + str((2*x+w)/2), 
                        (int(((2*x+w)/2)-20), int(((2*y+h)/2))+20), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.35, (255, 255, 255))
            cv2.putText(imageFrame, 
                        "Y: " + str((2*y+h)/2), 
                        (int(((2*x+w)/2)-20), int(((2*y+h)/2))+30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.35, (255, 255, 255))
            cv2.putText(imageFrame, "Red Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))    
  
    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 500):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h),
                                       (0, 255, 0), 2)
            imageFrame = cv2.circle(imageFrame,
                                    (int((2*x+w)/2), int((2*y+h)/2)),
                                    5,(255,255,255), cv2.FILLED)

            cv2.putText(imageFrame, 
                        "X: " + str((2*x+w)/2), 
                        (int(((2*x+w)/2)-20), int(((2*y+h)/2))+20), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.35, (255, 255, 255))
            cv2.putText(imageFrame, 
                        "Y: " + str((2*y+h)/2), 
                        (int(((2*x+w)/2)-20), int(((2*y+h)/2))+30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.35, (255, 255, 255))
            cv2.putText(imageFrame, "Green Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 255, 0))
  
    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 500):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)
            imageFrame = cv2.circle(imageFrame,
                                    (int((2*x+w)/2), int((2*y+h)/2)),
                                    5,(255,255,255), cv2.FILLED)

            cv2.putText(imageFrame, 
                        "X: " + str((2*x+w)/2), 
                        (int(((2*x+w)/2)-20), int(((2*y+h)/2))+20), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.35, (255, 255, 255))
            cv2.putText(imageFrame, 
                        "Y: " + str((2*y+h)/2), 
                        (int(((2*x+w)/2)-20), int(((2*y+h)/2))+30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.35, (255, 255, 255))
            cv2.putText(imageFrame, "Blue Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))
                        #hello
              
    # Program Termination
    cv2.imshow("Mask", green_mask)
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break
import cv2
import numpy as np
import argparse
import imutils
import time
import copy
import math
from math import pi as PI
from math import *

#Import the VideoCamera feed
cap = cv2.VideoCapture(0)

#Set the HSV bounds for the two tags
lower_blue = np.array([90, 0, 50])
upper_blue = np.array([140, 255, 255])
lower_white = np.array([0,0,250])
upper_white = np.array([180,255,255])

#Functions for computing the heading 
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return a
#Function for computing the heading
def getAngleBetweenPoints(x_orig, y_orig, x_landmark, y_landmark):
    deltaY = y_landmark - y_orig
    deltaX = x_landmark - x_orig
    return angle_trunc(atan2(deltaY, deltaX))

while (1):
    #Read in the frame and create a copy for each circle
    _, frame = cap.read()
    frame2 = copy.deepcopy(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        #Convert to HSV values
    hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)      #Do the same for the white circle


    mask = cv2.inRange(hsv, lower_blue, upper_blue)   #Create the Mask and filter it so its nice
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    kernel = np.ones((15, 15), np.float32) / 225
    res = cv2.filter2D(res, -1, kernel)

    #Look for Contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    #Initialize because sometimes we get errors when they dont exist yet
    x2=0
    y2=0
    y=0
    x=0
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX



    #Do the Same for whiteCircle
    mask2 = cv2.inRange(hsv2, lower_white, upper_white)
    mask2 = cv2.erode(mask2, None, iterations=2)
    mask2 = cv2.dilate(mask2, None, iterations=2)
    res2 = cv2.bitwise_and(frame2, frame2, mask=mask2)

    kernel2 = np.ones((15, 15), np.float32) / 225
    smoothed2 = cv2.filter2D(res2, -1, kernel2)
    res2 = cv2.medianBlur(res2, 15)

    cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts2 = imutils.grab_contours(cnts2)
    center = None

    # only proceed if at least one contour was found
    if len(cnts2) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts2, key=cv2.contourArea)
        ((x2, y2), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x2), int(y2)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)




    #Position Of robot to be Outputted
    Robot_x = ((x2-60)* 711) / 540
    Robot_y = ((450-y2)*482) / 450
    
    #Position of the robot head (used for finding the heading)
    Head_x = x-60
    Head_y = 450-y
    
    #Calculate the heading 
    Angle = getAngleBetweenPoints(Robot_x,Robot_y,Head_x,Head_y)

    #Output Lines, Shapes,and Text
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, 'X:' + str(round(Robot_x)), (0, 30), font, 1, (200, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, 'Y:' + str(round(Robot_y)), (200, 30), font, 1, (200, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, 'H:' + str(Angle), (350, 30), font, 1, (200, 255, 255), 2, cv2.LINE_AA)

    cv2.line(frame, center, (int(x), int(y)), (255,0, 0), 2)
    cv2.rectangle(frame, (60, 60), (600, 450), (0, 255, 0), 5)

    #Display Images
    cv2.imshow('Original', frame)
    cv2.imshow('res', res)
    cv2.imshow('res2',res2)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()

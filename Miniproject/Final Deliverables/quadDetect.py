#New Quad Detect

import numpy as np
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

#Initializes camera and aruco dictionary
cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

#Continously takes pictures
while (True):

    ret,frame = cap.read()
    corners,ids,rejects = cv2.aruco.detectMarkers(frame,aruco_dict)
    cv2.imshow('Live Video',frame)
    #Checks if markers are detected
    if ids is None:
        print("No markers found")


    else:
        #Defines corners of marker if present
        corners = np.array(corners).reshape(4,2)
        top_left,top_right,bottom_right,bottom_left = corners

        top_left[0] = int(top_left[0])
        print(top_left[0])
        top_left[1] = int(top_left[1])
        top_right[0] = int(top_right[0])
        top_right[1] = int(top_right[1])
        bottom_right[0] = int(bottom_right[0])
        bottom_right[1] = int(bottom_right[1])
        bottom_left[0] = int(bottom_left[0])
        bottom_left[1] = int(bottom_left[1])

        centerX = (top_left[0]+top_right[0])/2
        centerY = (top_left[1]+top_right[1])/2

        #Checks Quadrants
        if centerX > 320 and centerY >240:
            frame = cv2.putText(greyscaled,'3pi/2',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,0,0),3)
            quadrant = '3pi/2';
        elif centerX < 320 and centerY >240:
            frame = cv2.putText(frame,'pi',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,0,0),3)
            quadrant = 'pi';
        elif centerX < 320 and centerY <240:
            frame = cv2.putText(frame,'pi/2',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,0,0),3)
            quadrant = 'pi/2';
        elif centerX > 320 and centerY <240:
            frame = cv2.putText(frame,'0',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,0,0),3)
            quadrant = '0';
        cv2.imshow('Live Video',frame)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

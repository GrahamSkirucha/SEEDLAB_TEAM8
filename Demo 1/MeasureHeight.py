 #Angle Detect

import numpy as np
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera
import time, math


    
#Initializes camera and aruco dictionary
cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

#Continously takes pictures
while (True):
    
    ret,frame = cap.read()
    #frame.resolution = (640, 320)
    #greyscaled = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,blank = cv2.aruco.detectMarkers(frame,aruco_dict)
    

    #Checks if markers are detected
    if ids is None:
        print("No markers found")

    
    else:
        #(29592,1944)
        #Defines corners of marker if present
        corners = np.reshape(corners,(4,2))
        top_left,top_right,bottom_right,bottom_left = corners
        top_left[0] = int(top_left[0])
        top_left[1] = int(top_left[1])
        top_right[0] = int(top_right[0])
        top_right[1] = int(top_right[1])
        bottom_right[0] = int(bottom_right[0])
        bottom_right[1] = int(bottom_right[1])
        bottom_left[0] = int(bottom_left[0])
        bottom_left[1] = int(bottom_left[1])
        
        # Estimate the pose of the marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, ids, blank)

    # Calculate the height of the marker
        height = tvecs[0][0][1]  # Height is the y-coordinate of the translation vector

        # Draw the marker and the height on the image
        #cv2.aruco.drawDetectedMarkers(frame, corners)
        cv2.putText(frame, height, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
      
    #Displays live video
    cv2.imshow('Live Video',frame)  

   
cap.release()
cv2.destroyAllWindows()

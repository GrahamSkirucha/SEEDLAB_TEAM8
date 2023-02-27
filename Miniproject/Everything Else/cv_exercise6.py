#Exercise 6

import numpy as np
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera
import time


def cv_exercise6():
    
    #Initializes camera and aruco dictionary
    cap = cv2.VideoCapture(0)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
   
    #Continously takes pictures
    while (True):
        
        ret,frame = cap.read()
        greyscaled = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners,ids,rejectedImgPoints = cv2.aruco.detectMarkers(greyscaled,aruco_dict)
        
    
        #Checks if markers are detected
        if ids is None:
            print("No markers found")

        
        else:
            
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
                
            #Finds the angle to the marker
            angle = ((top_left[0]+top_right[0])/2-320)*(53.5/640)
           
            #Displays text
            print("Aruco marker ID detected:",ids)
            print("Angle from marker:",angle)
            greyscaled = cv2.putText(greyscaled,str(angle),(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,255),3)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break
          
        #Displays live video
        cv2.imshow('Live Video',greyscaled)  
 
       
    cap.release()
    cv2.destroyAllWindows()

cv_exercise6()
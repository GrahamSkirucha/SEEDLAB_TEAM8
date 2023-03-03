#Angle Detect

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
    #greyscaled = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,blank = cv2.aruco.detectMarkers(frame,aruco_dict)
    

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
        angle = -1*angle
#        centerX = (top_left[0]+top_right[0])/2
#        centerY = (top_left[1]+top_right[1])/2
#        if centerX > 640 and centerY >0:
#            cv2.putText(greyscaled,'0',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,255),3)
#        elif centerX < 640 and centerY >0:
#            cv2.putText(greyscaled,'pi/2',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,255),3)
#        elif centerX < 640 and centerY <0:
#            cv2.putText(greyscaled,'pi',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,255),3)
#        elif centerX > 640 and centerY <0:
#            cv2.putText(greyscaled,'3pi/2',(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,255),3)
#
        #Displays text
#        print("Aruco marker ID detected:",ids)
#        print("Angle from marker:",angle)
        frame = cv2.putText(frame,str(angle),(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,0,0),3)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break
      
    #Displays live video
    cv2.imshow('Live Video',frame)  

   
cap.release()
cv2.destroyAllWindows()

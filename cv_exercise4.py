#Exercise 4

import numpy as np
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

def cv_exercise4(fileName):
    
    #Define aruco dictionary and load image
    
    aruco_dict = cv2.aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    image = cv2.imread(fileName)
    greyscaled = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
   
    #Detect aruco markers
    
    (corners,ids,rejectedImgPoints)= cv2.aruco.detectMarkers(greyscaled,aruco_dict,parameters = parameters)
    
    
    
    if ids is None:
        print("No markers found")
        return

    ids = ids.flatten()
    
    markers = zip(corners,ids)
    
    #looping through markers and ids
    id_pos = 100
    for (c,i) in markers:

        cv2.putText(greyscaled,str(i),(100,id_pos),1,5,(0,0,0),3,1,False)

        print("Aruco marker ID detected:",i)
        id_pos = id_pos + 400
        
        #Showing the image
    cv2.imshow('image',greyscaled)        
    cv2.waitKey(0)        
    cv2.destroyAllWindows()

cv_exercise4('noMarkers.png')    
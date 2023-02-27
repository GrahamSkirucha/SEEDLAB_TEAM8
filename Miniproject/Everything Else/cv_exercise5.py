#Exercise 5

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

#Initializing the video capture and aruco dictionary
cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

#Loop through multiple images
while (True):
    
    ret, frame = cap.read()
    
    #Grayscale
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    corners,ids,blank = cv2.aruco.detectMarkers(gray,aruco_dict)
    
    #Checking for marker

    if ids is None:
        print("No markers found")
    else:
        print ("Marker ID:",ids)
        gray = cv2.putText(gray,str(ids),(100,400),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,255),3)
    
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
    
    #Displaying live video
    cv2.imshow('Live Video',gray)
#    cv2.waitKey(0)        
#    cv2.destroyAllWindows()
cap.release()
cv2.destroyAllWindows()
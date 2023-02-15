#Mini Project (CV)
 
import numpy as np
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray

from time import sleep
from picamera import PiCamera

def setWhiteBalance():
    fileName = 'test.jpg'
    camera = PiCamera(resolution = (1280,720),framerate = 30)
    camera.iso = 100
    sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains

    camera.awb_mode = 'off'
    camera.awb_gains = g
    
    camera.capture_sequence(['image%02d.jpg'% i for i in range(10)])
    
    
    camera.capture(fileName)
    image = cv2.imread(fileName)

    cv2.putText(image,str(g),(100,400),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),3)
    cv2.imshow('Reference Image',image)
    cv2.waitKey(0)

           
    cv2.destroyAllWindows()

setWhiteBalance()
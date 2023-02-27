#Exercise 3

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

def cv_exercise3(fileName):
    camera = PiCamera()
    camera.resolution = (1280,1118)
    time.sleep(0.1)
    
    #Takes image
    camera.capture(fileName)
    image = cv2.imread(fileName)
    
    #Converts image to grayscale
    greyscaled = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    cv2.imwrite(fileName,greyscaled)
    
    #Displays grayscaled image
    cv2.imshow("Greyscaled image",greyscaled)

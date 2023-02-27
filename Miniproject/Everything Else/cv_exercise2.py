#Exercise 2

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

def cv_exercise2(fileName):
    camera = PiCamera()
    camera.resolution = (1280,1118)
    time.sleep(0.1)
    #Takes picture
    camera.capture(fileName)
    image = cv2.imread(fileName)
   
    #Scales new dimensions of image by 0.5
    width = int(image.shape[1]*.5)
    height = int(image.shape[0]*.5)
    dim = (width,height)
    
    #Defines new resized image
    resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)    
    cv2.imwrite(fileName,resized_image)
    cv2.imshow("Resized image",resized_image)

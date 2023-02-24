#Exercise 1

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

if __name__ == '__main__':
    
    #Prompting for filename
    filename = input("Please input a filename to store an image> ")
    camera = PiCamera()

    time.sleep(0.1)
    
    camera.resolution = (64,64)
    camera.start_preview()
    time.sleep(5)
    camera.stop_preview()
    
    
    print("Capturing Image...")
    try:
        #Takes the image
        camera.capture(filename)

    except:
        print("Failed to capture")

    print("Saving image "+filename)
    
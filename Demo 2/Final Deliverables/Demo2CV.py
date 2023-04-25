 #Angle Detect

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import busio
import board
import serial
import cv2
import numpy as np
import smbus
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera

import numpy as np
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera
import time, math


    
#Initializes camera and aruco dictionary
cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

ser = serial.Serial('/dev/ttyACM0', 115200, timeout = .001)

#paramters used
point_set = 0
current_position = 0

#i2c code that is not used but can be if wanted just slows things down
#i2c = busio.I2C(board.SCL, board.SDA)
#lcd_columns = 16
#lcd_rows = 2
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

counter = 0
angle = 0
distance = 0
#Continously takes pictures
while (True):
    
    ret,frame = cap.read()
    #frame.resolution = (640, 320)
    #greyscaled = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,blank = cv2.aruco.detectMarkers(frame,aruco_dict)
    

    #Checks if markers are detected
    #if ids is None:
        #print("No markers found")

    
    #else:
    if ids is not None:    
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
        
        #area = (top_right[0] - top_left[0])*(bottom_right[0] - bottom_left[0])
        #areaRatio = 204800 / area
        
        pixel = abs(((top_left[1] - bottom_left[1])+(top_right[1] - bottom_right[1]))/2)
        
        distance = 161.87*(pixel)**(-1.044)
        print(pixel)
        print(distance)
        #Finds the angle to the marker
        #58 or 65
        angle = ((top_left[0]+top_right[0])/2-320)*(65/640)
        angle = -1*angle
        
        #angleRad = math.pi / 180
        #d = areaRatio / math.cos(abs(angleRad))
        frame = cv2.putText(frame,str(angle),(320,100),cv2.FONT_HERSHEY_SIMPLEX,3,(255,0,0),3)
        frame = cv2.putText(frame,str(distance),(320,200),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),3)
    if ((angle != 0) and (distance <= 6.0)):
        #send_angle = str(angle) + '\n'
        angle = round(angle, 1)
        distance = round(distance, 2)
        send_angle = str(angle) + ','
        send_distance = str(distance) + '\n'
        print(send_angle)
        print(send_distance)
        send_total = send_angle + send_distance
        ser.write((send_total).encode())
        #break
        print("End of print")
        angle = 0
        distance = 0
        #ser.write((send_distance).encode())
    
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
      
    #Displays live video
    cv2.imshow('Live Video',frame)  

   
cap.release()
cv2.destroyAllWindows()

#Graham Skirucha

#Demo 1 Serial Communication Code with I2C display integrated
#Need to ADD CV to have aruco for functionality

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import time
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
import time


#ser = serial.Serial('/dev/ttyACM0', 38400, timeout = .001)



point_set = 0
current_position = 0

cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

i2c = busio.I2C(board.SCL, board.SDA)

lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
counter = 0
angle = 0
while True:
    #place CV aruco camera settings in here from Ryans code
    ret,frame = cap.read()
    #greyscaled = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,blank = cv2.aruco.detectMarkers(frame,aruco_dict)
#    while ser.in_waiting:
#        current_position = ser.readline()
#
#        print('position: ', current_position)
#        print(point_set)

    #place CV detection code here from Ryan may need more parameters TBD
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

 

    #print('send setpoint')
    point_set = str(angle) + '\n'
    #print('Writing')
    #ser.write(point_set.encode())
    counter += 1
    #print('sent')
    # wait before requesting position again
    #time.sleep(.1)
    #print("recieved")

    if counter == 7:
        lcd.clear()
        # print('message')
        counter = 0
        #make sure to change to angle and distance
        lcd.message = 'Setpoint: {}\n'.format(point_set)
    
  #unsure if needed
    cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
#ser.close()





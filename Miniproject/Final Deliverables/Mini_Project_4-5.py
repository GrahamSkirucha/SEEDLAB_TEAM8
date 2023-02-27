#Graham Skirucha // System Intergartion Mini Project
#Section 4.5 Code for Python
#Control Based on setpoints decided by Aruco Frames
#Send setpoints to Arduino

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import time
import busio
import board
import serial
import cv2
import numpy as np
import smbus


ser = serial.Serial('/dev/ttyACM0', 38400, timeout = .001)
#i2c = board.I2C()
#bus = smbus.SMBus(1)
#Setpoint Library Key
#Top "0" = 0
#Right "1" = np.pi/2
#Bottom "2" = 3*np.pi/2
#Left "3" = np.pi
point_set = 0
current_position = 0

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

#def ReadfromArduino():
#    while (ser.in_waiting > 0):
#        try:
#            line = ser.readline()
#            print("serial output : ", line)
#        except:
#            print("Communication Error")
#I2C Setup for Pi to display info on LCD
i2c = busio.I2C(board.SCL, board.SDA)

lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
cap = cv2.VideoCapture(0);
counter = 0
while True:
    ret, frame = cap.read()
    
    corners, ids, _= cv2.aruco.detectMarkers(frame, aruco_dict, parameters = aruco_params)
    
    while ser.in_waiting:
        current_position = ser.readline()
        
        print('position: ',current_position)
        print(point_set)
    
    if ids is not None:
        marker_id = ids[0][0]
        marker_corners = corners[0][0]
        marker_x, marker_y = np.mean(marker_corners, axis = 0)
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        if marker_x <frame.shape[1]/3:
            point_set = 1
        elif marker_x < 2*frame.shape[1]/3:
            if marker_y < frame.shape[0]/2:
                point_set = 0
            else:
                point_set = 2
        else:
            point_set = 3
    

        #ser.write(b'p')
        print('send setpoint')
        #bus.write_byte(0x04, point_set)
        point_set = str(point_set) + '\n'
        print('Writing')
        ser.write(point_set.encode())
        counter += 1
        print('sent')
    #wait before requesting position again
        time.sleep(.1)
    
        
        
#        while ser.in_waiting < 2:
#            time.sleep(.01)
#            print('waiting')
        print('recieved')
        
        #print('LCD')
        #
        if counter == 7:
            lcd.clear()
        #print('message')
            counter = 0
            lcd.message = 'Setpoint: {}\nPosition: {}'.format(point_set, current_position)
        #print('printed')
    #lcd.clear()
    #lcd.message = 'Completed'
    
    cv2.imshow("frame", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
ser.close()

_________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
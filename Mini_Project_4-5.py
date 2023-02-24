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

ser = serial.Serial('/dev/ttyACM0', 250000)

#Setpoint Library Key
#Top "0" = 0
#Right "1" = np.pi/2
#Bottom "2" = 3*np.pi/2
#Left "3" = np.pi
point_set = 0
current_position = 0

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()


#I2C Setup for Pi to display info on LCD
i2c = busio.I2C(board.SCL, board.SDA)
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
cap = cv2.VideoCapture(0);

while True:
    ret, frame = cap.read()
    
    corners, ids, _= cv2.aruco.detectMarkers(frame, aruco_dict, parameters = aruco_params)
    
    if ids is not None:
        marker_id = ids[0][0]
        marker_corners = corners[0][0]
        marker_x, marker_y = np.mean(marker_corners, axis = 0)
        
        if marker_x <frame.shape[1]/3:
            point_set = 1
        elif marker_x < 2*frame.shape[1]/3:
            if marker_y < frame.shape[0]/2:
                point_set = 0
            else:
                point_set = 2
        else:
            point_set = 3
        
        #Need to calculate based off motor control code and CV
#        current_position = 0
#    
#        if current_position < 250:
#            point_set = 0
#        elif current_position < 500:
#            point_set = 1
#        elif current_position < 750:
#            point_set = 2
#        else:
#            point_set = 3

        ser.write(b'p')
        print('send setpoint')
        point_set_byte = point_set.to_bytes(1, byteorder = 'little')
        print('Writing')
        ser.write(point_set_byte)
        print('sent')
    #wait before requesting position again
        time.sleep(.1)
    
        
        
#        while ser.in_waiting < 2:
#            time.sleep(.01)
#            print('waiting')
        print('recieved')
        position_bytes = ser.read(2)
        current_position = int.from_bytes(position_bytes, byteorder = 'little')
        print(current_position)
        print(point_set)
        #current_position = 0  
#        positionString = ''
#        if current_position == 0:
#            current_position = 0
#            positionString = '0π '
#        elif current_position == 1:
#            current_position = 90
#            positionString = 'π/2' 
#        elif current_position == 2:
#            current_position = 180
#            positionString = 'π' 
#        else:
#            current_position = 270
#            positionString = '3π/2' 
        print('LCD')
        lcd.clear()
        print('message')
        lcd.message = 'Setpoint: {}\nPosition: {}'.format(point_set, current_position)
        print('printed')
    lcd.clear()
    #lcd.message = 'Completed'
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("frame", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
ser.close()

#point_set = 0
#current_position = 0
#
#
#
#
##I2C Setup for Pi to display info on LCD
#i2c = busio.I2C(board.SCL, board.SDA)
#lcd_columns = 16
#lcd_rows = 2
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

#Find values to setpoint and set them for arduino translation
#while True:
#    #Need to calculate based off motor control code and CV
#    current_position = 0
#    
#    if current_position < 250:
#        point_set = 0
#    elif current_position < 500:
#        point_set = 1
#    elif current_position < 750:
#        point_set = 2
#    else:
#        point_set = 3
#        
#    point_set_byte = point_set.to_bytes(1, byteorder = 'little')
#    ser.write(point_set_byte)
#    
#    #wait before requesting position again
#    time.sleep(.1)
#    
#    ser.write(b'p')
#    
#    while ser.in_waiting < 2:
#        time.sleep(.01)
#        
#    position_bytes = ser.read(2)
#    current_position = int.from_bytes(position_bytes, byteorder = 'little')
#    
#    lcd.clear()
#    lcd.message = 'Setpoint: {}\nPosition: {}'.format(point_set, current_position)
    
    
#ser.close()





_________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
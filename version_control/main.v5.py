#//////////////////////////////IMPORTING LIBRARIES////////////////////
#////////////////////////////////////////////////////////////////////

import RPi.GPIO as GPIO
from picamera import PiCamera
from time import sleep
import time  
from datetime import datetime
import datetime as dt
import os
import serial
import csv
import arduino_readings
import itertools
import math 

# Import Raspberry Pi GPIO library
# Ignore warning for now
#///////////////////////////////////SETUP////////////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////

GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

powerstate = False
f=open('/home/pi/Desktop/Saves/Sensors/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
writer = csv.writer(f)

# transceiver = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
# transceiver.reset_input_buffer()

#////////////////////////////VARIABLES/////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////

t = 1800
temp_speed = 0
max_speed = 0
distance = 8000
cadance = 0
timeout=0;

#///////////////////////CAMERA SETUP/////////////////////////////////
#///////////////////////////////////////////////////////////////////

camera = PiCamera()
camera.rotation = 180

#///////////////////////////////////MAIN////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////

while True:
    if GPIO.input(7) == GPIO.HIGH and powerstate == False :
        
        #/////////////////////////////FILE SETUP/////////////////////////////////////////////
        #/////////////////////////////////////////////////////////////////////////////////
        
        f=open('/home/pi/Desktop/Saves/Sensors/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
        camera.start_recording('/home/pi/Desktop/Camera Videos /Vid_ ' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.h264')
        writer = csv.writer(f)
        time_start = time.time()
        
        #/////////////////////////Starting Camera and time/////////////////////////////// 
        #///////////////////////////////////////////////////////////////////////////////
        
        camera.start_preview()
        camera.annotate_background = False
        camera.annotate_text_size = 32
        start = dt.datetime.now()
        powerstate = True
        
    if GPIO.input(7) == GPIO.HIGH:
        
        #/////////////////////////////DATALOGING////////////////////////////////////
        #///////////////////////////////////////////////////////////////////////////
        
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        arduino_readings.main()
        line = ('{0}, {1}, {2}, {3}, {4}, {5}, {6}' .format(current_time, arduino_readings.main.wheel_centre, arduino_readings.main.wheel_left, arduino_readings.main.wheel_right, arduino_readings.main.gear,arduino_readings.main.Total_Speed, arduino_readings.main.error_response))
        writer.writerow([line])
       #/////////////////////SETTING MAXSPEED///////////////////////////////////
       #////////////////////////////////////////////////////////////////////////
#         if time.time()-timeout>300:
#                 transceiver.write(line)
#                 timeout=time.time()
#         
       
       #/////////////////////SETTING MAXSPEED///////////////////////////////////
       #////////////////////////////////////////////////////////////////////////
        
        #temp_speed=float(arduino_readings.data.Total_Speed)
        
        #if temp_speed > max_speed :
         #       max_speed = temp_speed
       
       #///////////////////////////MAIN BIKE CALCULATIONS///////////////////////////
       #////////////////////////////////////////////////////////////////////////////
            
        gearratio = [0.1354,0.1185,0.1058,0.0931,0.0847,0.0804,0.0677]
        cadance = float(arduino_readings.main.crank)  * gearratio[arduino_readings.main.gear-1]
#         distance = 8000 - ((time.time()-time_start) * float(arduino_readings.data.Total_Speed))
        
        #///////////////////////////////////OVERLAY///////////////////////////////////////
        #///////////////////////////////////////////////////////////////////////////////
        
        arduino_readings.main()
        Cam_Speed = str(arduino_readings.main.Total_Speed)
        Cam_Gear = str(arduino_readings.main.gear)
#         Cam_Temp = str(arduino_readings.data.Temperature)
#         Cam_Humd = str(arduino_readings.data.Humidity)
#         Cam_Press = str(arduino_readings.data.Pressure)
#         Cam_MaxSpD = str(max_speed)
#         Cam_cadance = str(round(cadance,2))
        Cam_distance = str(distance)
        #Cam_crank = str(arduino_readings.data.crank)
        
        #//////////////////////MODES FOR THE HUD//////////////////////////////////////////////////
        #////////////////////////////////////////////////////////////////////////////////////////
        
#         i = ' Speed: ' + Cam_Speed + ' Gear: '+ Cam_Gear +'                            '+ ' Temp: '+Cam_Temp+'                                               '+' Humidity: '+Cam_Humd+'                                          '+' Pressure: '+Cam_Press
#         j = ' Gear: '+ Cam_Gear +'                                     '+ ' Temp: '+Cam_Temp+'                                               '+' Humidity: '+Cam_Humd+'                                          '+' Max Speed: '+Cam_MaxSpD 
        z = ' Cadance: ' +str(arduino_readings.main.crank) +     ' RPM: ' + str(arduino_readings.main.wheel_centre) +'                            '+ str(arduino_readings.main.lat)
        x = ' Cadance: ' +str(arduino_readings.main.crank) +     ' RPM: ' + str(arduino_readings.main.wheel_centre) + '           '+arduino_readings.main.error_response +'            '+ str(arduino_readings.main.lat)
        #DO NOT TOUCH 28  or 48 spaces for test to be at the left hand and right hand corner of the screeen 
        
        #///////////////////////SETTING THE MODE FOR THE HUD///////////////////////////////////////
        #/////////////////////////////////////////////////////////////////////////////////////////
        
        camera.annotate_text = x
        
        
    if GPIO.input(7) == GPIO.LOW and powerstate == True :
        
        #/////////////////////////SAVING AND CLOSING FILES/////////////////////////////////
        #/////////////////////////////////////////////////////////////////////////////////
        
        end_time = time.time()
        f.close()
        camera.stop_recording()        
        camera.stop_preview()
        powerstate = False
        exit()

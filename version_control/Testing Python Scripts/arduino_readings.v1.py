#//////////////////////IMPORTING LIBRARIES/////////////////////////
#//////////////////////////////////////////////////////////////////

import time
import serial

#///////////////////////ARDUNIO SETUP///////////////////////////////
#///////////////////////////////////////////////////////////////////

# esphatch = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
# esphatch.reset_input_buffer()

espbike = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
espbike.reset_input_buffer()

#gear = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#gear.reset_input_buffer()

# line = esp-hatch.readline().decode('utf-8').rstrip()
# Sensors_list = line.split(",")

#/////////////////////MAIN//////////////////////////////
#///////////////////////////////////////////////////////

def data():
    #////////////////////SETUP////////////////////////////////
    #/////////////////////////////////////////////////////////
    
    line = espbike.readline().decode('utf-8').rstrip()
    Sensors_list = line.split(",")
    if(len(Sensors_list)!=8):
       for x in range(8-len(Sensors_list)):
           Sensors_list.append(x)
           
    #/////////////////SENSOR LIST///////////////////////////
    #///////////////////////////////////////////////////////
    if Sensors_list[0] == 'inf':
        Sensors_list[0] = 0
    data.wheel_left = Sensors_list[0]
    data.wheel_right = Sensors_list[1]
    data.wheel_centre = Sensors_list[2]
    #data.crank = Sensors_list[3]
    data.gear = 3
    data.Temperature = Sensors_list[4]
    data.Humidity = Sensors_list[5]
    data.Pressure = Sensors_list[6]
    data.Total_Speed = Sensors_list[3]
   
    
    

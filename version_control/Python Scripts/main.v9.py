#//////////////////////////////IMPORTING LIBRARIES////////////////////
#////////////////////////////////////////////////////////////////////

import RPi.GPIO as GPIO
from picamera import PiCamera
import time  
from datetime import datetime
import datetime as dt
import csv
from microcontroller_readings2 import SensorDataProcessor
import os

#///////////////////////////////////SETUP////////////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////

GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

powerstate = False
f=open('/home/pi/Desktop/Saves/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
writer = csv.writer(f)


#////////////////////////////VARIABLES/////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////

temp_speed = 0
max_speed = 0
distance = 8000
line_count = 0
printed_times=0
auto_port_count = 0
time_start=0
file_open = True
debounce = True
millis_start = 0
ports_incomplete = True
transmit_count = 0
calculated_times=0
distance_calculation_interval=15 #in milliseconds
distance_traveled=0
time_start =float(time.perf_counter())*1000
time_last=float(time.perf_counter())*1000
time_last_ts=float(time.perf_counter())*1000
window_size = 10
ts_list = []
display_ts = 0



#///////////////////////CAMERA SETUP/////////////////////////////////
#///////////////////////////////////////////////////////////////////

camera = PiCamera()
camera.rotation = 0
camera.contrast = 75 
camera.image_effect = "saturation" 

# setup function
sensor_data_processor = SensorDataProcessor()

# Boot auto porting on startup
if ports_incomplete:
        auto_port_count = 0
        ports_data = sensor_data_processor.auto_port()
        port_status = "devices missing"
        if sensor_data_processor.check_port_complete(ports_data):
            ports_incomplete = False
            port_status = "devices all connected"

print("ports complete status:")
print(not ports_incomplete)
#///////////////////////////////////MAIN////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////


def moving_average(a, n):
    ret = sum(a)
    return ret/n
    

while True:
    millis = float(time.perf_counter())*1000
    
    #Other calculations
    #total speed in kph
    sensor_data_processor.ts = round((float(sensor_data_processor.c)*1.47655)*(60/1000),5)
    
    #moving average of total speed
    if len(ts_list) < window_size:
        ts_list.append(sensor_data_processor.ts)
    if len(ts_list) == window_size:
        display_ts = round(moving_average(ts_list,window_size))
        ts_list = ts_list[:1]

    if (millis - time_last >=distance_calculation_interval): #Calculate distance in m from speed
        calculated_times += 1
        distance_traveled=distance_traveled+(sensor_data_processor.ts/3.6666)*((millis-time_last)/1000)
        time_last=millis
        sensor_data_processor.dt = int(distance_traveled)
    
    if GPIO.input(7) == GPIO.LOW and powerstate == False :
        if not file_open:
            file_open = True
        time.sleep(2)
        debounce = True
        #/////////////////////////////FILE SETUP/////////////////////////////////////////////
        #/////////////////////////////////////////////////////////////////////////////////
        f=open('/home/pi/Desktop/Saves/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
        file_open = True
        camera.start_recording('/home/pi/Desktop/Camera Videos/Vid_ ' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.h264')
        writer = csv.writer(f)
        #/////////////////////////Starting Camera and time/////////////////////////////// 
        #///////////////////////////////////////////////////////////////////////////////
        camera.start_preview()
        camera.annotate_background = True
        camera.annotate_text_size = 32
        start = dt.datetime.now()
        millis_start = time.perf_counter()*1000
        powerstate = True
        
    if GPIO.input(7) == GPIO.LOW:
        if (millis >(20*printed_times+time_start)):
            printed_times += 1
            line_count += 1
            auto_port_count += 1
            transmit_count += 1
            
            # run any calculations before this function is called
            data_stream = sensor_data_processor.process()
            data_stream.insert(0, str(datetime.now().strftime('%H_%M_%S_%f'))[:-3])
            
            writer.writerow(data_stream)
            
            if transmit_count >= 20:
                transmit_count = 0
                sensor_data_processor.transmit()
             
            if line_count >= 10:
                line_count = 0
                os.fsync(f.fileno())
                f.flush()
             
            if auto_port_count == 1000 and ports_incomplete:
                auto_port_count = 0
                ports_data = sensor_data_processor.auto_port()
                port_status = "devices missing"
                if sensor_data_processor.check_port_complete(ports_data):
                    ports_incomplete = False
                    port_status = "devices all connected"
                    
                sensor_data_processor.dt = distance_traveled
                
            #//////////////////////MODES FOR THE HUD//////////////////////////////////////////////////
            #////////////////////////////////////////////////////////////////////////////////////////
            #speed = round(float(sensor_data_processor.c)*0.000508,1)
            standard_overlay = 'Cadance: ' +str(sensor_data_processor.cr) + '     ' +    ' KPH: ' +str(display_ts) + '     '+ 'Gear: '+ str(round(sensor_data_processor.g)) + '     '+'Distance: ' + str(sensor_data_processor.dt) # + '     ' + 'Batt: '+str(sensor_data_processor.bp)
            analysis_overlay_line1 = 'c: ' +str(sensor_data_processor.c) + ' ' + ' l: ' + str(sensor_data_processor.l) +' '+ 'r: '+str(sensor_data_processor.r) +' '+ 'cr: '+str(sensor_data_processor.cr)  +' '+ 's: '+str(sensor_data_processor.s)+' '+ 'ts: '+str(sensor_data_processor.ts)+' '+ 'sa: '+str(sensor_data_processor.sa)+' '+ 'g: '+str(sensor_data_processor.g)+' '+ 'bg: '+str(sensor_data_processor.bg) 
            analysis_overlay_line2 = 'ax: '+str(sensor_data_processor.ax)+' '+ 'ay: '+str(sensor_data_processor.ay)+' '+ 'az: '+str(sensor_data_processor.az)+' '+ 'vx: '+str(sensor_data_processor.vx)+' '+ 'vy: '+str(sensor_data_processor.vy)+' '+ 'vz: '+str(sensor_data_processor.vz)+'\n'+ 't: '+str(sensor_data_processor.t)+' '+ 'p: '+str(sensor_data_processor.p)+' '+ 'h: '+str(sensor_data_processor.h)+' '+ 'bp: '+str(sensor_data_processor.bp)+' '+ 'ba: '+str(sensor_data_processor.ba) + "dt" + str(sensor_data_processor.dt)
            analysis_overlay = analysis_overlay_line1 + '\n' + analysis_overlay_line2            
            #///////////////////////SETTING THE MODE FOR THE HUD///////////////////////////////////////
            #/////////////////////////////////////////////////////////////////////////////////////////    
#             camera.annotate_text = analysis_overlay +'\n'+ port_status
            camera.annotate_text = standard_overlay
        
    if GPIO.input(7) == GPIO.HIGH and powerstate == True and debounce == True:
        #/////DEBOUNCE IN CASE/////#
        time.sleep(1)
        debounce = False
        
    
    if GPIO.input(7) == GPIO.HIGH and powerstate == True and debounce == False:
        #/////////////////////////SAVING AND CLOSING FILES/////////////////////////////////
        #/////////////////////////////////////////////////////////////////////////////////
        time.sleep(2)
        end_time = time.time()
        f.close()
        file_open = False
        camera.stop_recording()
        camera.stop_preview()
        powerstate = False
        ports_incomplete = True
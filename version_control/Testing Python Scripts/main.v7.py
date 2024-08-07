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

#//TIMER FUNCTION//
#///////////////////////////////////SETUP////////////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////

GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

powerstate = False
f=open('/home/pi/Desktop/Saves/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
writer = csv.writer(f)



#////////////////////////////VARIABLES/////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////

t = 1800
temp_speed = 0
max_speed = 0
distance = 8000
cadance = 0
timeout=0
line_count = 0
auto_port_count = 0
print_time = 0
file_open = True
debounce = True
millis_start = 0
ports_incomplete = True
transmit_count = 0

#///////////////////////CAMERA SETUP/////////////////////////////////
#///////////////////////////////////////////////////////////////////

camera = PiCamera()
camera.rotation = 0
# camera.contrast = 75 // also causes freezing
# camera.image_effect = "saturation" // causes freezing

# setup function
sensor_data_processor = SensorDataProcessor()

if ports_incomplete:
        auto_port_count = 0
        ports_data = sensor_data_processor.auto_port()
        port_status = "devices missing"
        if sensor_data_processor.check_port_complete(ports_data):
            ports_incomplete = False
            port_status = "devices all connected"

print("ports incomplete status:")
print(ports_incomplete)
#///////////////////////////////////MAIN////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////

while True:
    if GPIO.input(7) == GPIO.HIGH and powerstate == False :
        if not file_open:
            file_open = True
        time.sleep(2)
        #/////////////////////////////FILE SETUP/////////////////////////////////////////////
        #/////////////////////////////////////////////////////////////////////////////////
        f=open('/home/pi/Desktop/Saves/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
        file_open = True
        camera.start_recording('/home/pi/Desktop/Camera Videos/Vid_ ' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.h264')
        time_start = time.time()
        writer = csv.writer(f)
        #/////////////////////////Starting Camera and time/////////////////////////////// 
        #///////////////////////////////////////////////////////////////////////////////
        camera.start_preview()
        camera.annotate_background = True
        camera.annotate_text_size = 32
        start = dt.datetime.now()
        millis_start = time.perf_counter_ns()
        powerstate = True
        
    if GPIO.input(7) == GPIO.HIGH:
        debounce = True
        if not file_open:
            file_open = True
            time.sleep(2)
            #/////////////////////////////FILE SETUP/////////////////////////////////////////////
            #/////////////////////////////////////////////////////////////////////////////////
            f=open('/home/pi/Desktop/Saves/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
            file_open = True
            camera.start_recording('/home/pi/Desktop/Camera Videos/Vid_ ' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.h264')
            time_start = time.time()
            writer = csv.writer(f)
            #/////////////////////////Starting Camera and time/////////////////////////////// 
            #///////////////////////////////////////////////////////////////////////////////
            camera.start_preview()
            camera.annotate_background = True
            camera.annotate_text_size = 32
            start = dt.datetime.now()
            millis_start = time.perf_counter_ns()
            powerstate = True
        #/////////////////////////////DATALOGING////////////////////////////////////
        #///////////////////////////////////////////////////////////////////////////
        millis_now = time.perf_counter_ns()
        if (millis_now - print_time) >=5000000:
            line_count += 1
            auto_port_count += 1
            transmit_count += 1
            now = datetime.now()
            stopwatch_start = time.perf_counter_ns()
            data_stream = sensor_data_processor.process()
            stopwatch_end = time.perf_counter_ns()
            print(data_stream)
            print(stopwatch_end - stopwatch_start)
            data_stream.insert(0,int((millis_now - millis_start)//1000000))
            writer.writerow(data_stream)
            
            if transmit_count == 20:
                transmit_count = 0
                sensor_data_processor.transmit()
            
            if line_count == 10:
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
                
                
            #//////////////////////MODES FOR THE HUD//////////////////////////////////////////////////
            #////////////////////////////////////////////////////////////////////////////////////////
            standard_overlay = 'Cadance: ' +str(sensor_data_processor.cr) + '     ' +    ' KPH: ' + str(sensor_data_processor.ts) +'     '+ 'Gear: '+str(sensor_data_processor.g) # + '     ' + 'Batt: '+str(sensor_data_processor.bp)
            analysis_overlay_line1 = 'c: ' +str(sensor_data_processor.c) + ' ' + ' l: ' + str(sensor_data_processor.l) +' '+ 'r: '+str(sensor_data_processor.r) +' '+ 'cr: '+str(sensor_data_processor.cr)  +' '+ 's: '+str(sensor_data_processor.s)+' '+ 'ts: '+str(sensor_data_processor.ts)+' '+ 'sa: '+str(sensor_data_processor.sa)+' '+ 'g: '+str(sensor_data_processor.g)+' '+ 'bg: '+str(sensor_data_processor.bg) 
            analysis_overlay_line2 = 'ax: '+str(sensor_data_processor.ax)+' '+ 'ay: '+str(sensor_data_processor.ay)+' '+ 'az: '+str(sensor_data_processor.az)+' '+ 'vx: '+str(sensor_data_processor.vx)+' '+ 'vy: '+str(sensor_data_processor.vy)+' '+ 'vz: '+str(sensor_data_processor.vz)+'\n'+ 't: '+str(sensor_data_processor.t)+' '+ 'p: '+str(sensor_data_processor.p)+' '+ 'h: '+str(sensor_data_processor.h)+' '+ 'bp: '+str(sensor_data_processor.bp)+' '+ 'ba: '+str(sensor_data_processor.ba)
            analysis_overlay = analysis_overlay_line1 + '\n' + analysis_overlay_line2            
            #///////////////////////SETTING THE MODE FOR THE HUD///////////////////////////////////////
            #/////////////////////////////////////////////////////////////////////////////////////////    
#             camera.annotate_text = analysis_overlay +'\n'+ port_status
            camera.annotate_text = analysis_overlay
            print_time = time.perf_counter_ns()
        
    if GPIO.input(7) == GPIO.LOW and powerstate == True and debounce == True:
        #/////DEBOUNCE IN CASE/////#
        time.sleep(1)
        debounce = False
        #/////////////////////////SAVING AND CLOSING FILES/////////////////////////////////
        #/////////////////////////////////////////////////////////////////////////////////
    if GPIO.input(7) == GPIO.LOW and powerstate == True and debounce == False:
        time.sleep(2)
        end_time = time.time()
        f.close()
        file_open = False
        camera.stop_recording()
        camera.stop_preview()
        powerstate = False
        ports_incomplete = True
        




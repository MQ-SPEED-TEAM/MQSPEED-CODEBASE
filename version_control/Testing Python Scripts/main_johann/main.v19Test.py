#//////////////////////////////IMPORTING LIBRARIES////////////////////
#////////////////////////////////////////////////////////////////////

import RPi.GPIO as GPIO
from PyQt6.QtWidgets import QApplication
from picamera2 import Picamera2, Preview, MappedArray
from picamera2.encoders import H264Encoder
import time  
from datetime import datetime
import datetime as dt
import csv
from microcontroller_readings_3 import SensorDataProcessor
import os
from multiprocessing import Process,Pipe,set_start_method
import pedal_readings_orig as prd
from picamera2.utils import Transform
import cv2
import numpy as np
from picamera2.outputs import FfmpegOutput
import sys



# Global overlay cache
cached_lines = []
frame_counter = 0
overlay_update_interval = 20  # Update overlay every 20 frames


def system():
    #///////////////////////////////////SETUP////////////////////////////////////////////
    #//////////////////////////////////////////////////////////////////////////////

    GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
    GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
    GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    powerstate = False
    BASE_PATH = '/home/mqspeed/Desktop/'
 
    run_once = True
    
    
    
    # start power multiprocess
    conn1, conn2 = Pipe() #conn1 is reading side and conn2 is writing side
    set_start_method("spawn")

    power_process = Process(target=prd.ant_main, args=(conn2,))
    power_process.start()
    
    time.sleep(3)
    #////////////////////////////VARIABLES/////////////////////////////////////
    #//////////////////////////////////////////////////////////////////////////////

    t = 1800
    temp_speed = 0
    max_speed = 0
    distance = 8000
    cadance = 0
    timeout=0
    line_count = 0
    printed_times=0
    auto_port_count = 0
    print_time = 0
    time_start=0
    file_open = True
    debounce = True
    debounce2 = True
    millis_start = 0
    ports_incomplete = True
    transmit_count = 0
    calculated_times=0
    calculation_time=0
    distance_calculation_interval=15 #in milliseconds
    distance_traveled=0
    time_start =float(time.perf_counter())*1000
    time_last=float(time.perf_counter())*1000
    time_last_ts=float(time.perf_counter())*1000
    old_value = 0
    window_size = 10
    ts_list = []
    display_ts = 0

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

    #///////////////////////CAMERA SETUP/////////////////////////////////
    #///////////////////////////////////////////////////////////////////
    
    
    # Setup camera settings
    encoder = H264Encoder()
    picam2 = Picamera2()
    video_config = picam2.create_video_configuration(main={"format": "XRGB8888", "size": (1640, 1232)},controls={"FrameRate": 30}, transform=Transform(rotation=0))
    picam2.configure(video_config)
    print("Starting camera preview")
    
    

    # Define overlay function
    # Overlay mode selection
    overlay_mode = "standard"
    # Text overlay settings
    
    def update_overlay_lines():
        global cached_lines


        overlay_dict = {
            "standard": [f"Cadence: {round(sensor_data_processor.cr)}   "
                         f"KPH: {round(sensor_data_processor.ts)}   "
                         f"Gear: {round(sensor_data_processor.g)}   "
                         f"Distance: {sensor_data_processor.dt}   "
                         f"Power: {round(sensor_data_processor.pr)}"],
            
            "basic": [f"Cadence: {round(sensor_data_processor.cr)}   "
                             f"KPH: {round(sensor_data_processor.ts)}   "
                             f"Gear: {round(sensor_data_processor.g)}"],

            "line1": [f"c: {sensor_data_processor.c}  l: {sensor_data_processor.l}  r: {sensor_data_processor.r}  "
                      f"cr: {sensor_data_processor.cr}  s: {sensor_data_processor.s}  ts: {sensor_data_processor.ts}  "
                      f"sa: {sensor_data_processor.sa}  g: {sensor_data_processor.g}  bg: {sensor_data_processor.bg}"],

            "line2": [f"rl: {sensor_data_processor.rl}  ph: {sensor_data_processor.ph}  yw: {sensor_data_processor.yw}",
                      f"t: {sensor_data_processor.t}  p: {sensor_data_processor.p}  h: {sensor_data_processor.h}",
                      f"bp: {sensor_data_processor.bp}  ba: {sensor_data_processor.ba}  dt: {sensor_data_processor.dt}"],

            "line3": [f"la: {sensor_data_processor.la}  lo: {sensor_data_processor.lo}  "
                      f"gs: {sensor_data_processor.gs}  al: {sensor_data_processor.al}  sn: {sensor_data_processor.sn}"],

            "analysis": [f"c: {sensor_data_processor.c}  l: {sensor_data_processor.l}  r: {sensor_data_processor.r}  "
                         f"cr: {sensor_data_processor.cr}  s: {sensor_data_processor.s}  ts: {sensor_data_processor.ts}  "
                         f"sa: {sensor_data_processor.sa}  g: {sensor_data_processor.g}  bg: {sensor_data_processor.bg}",
                         f"rl: {sensor_data_processor.rl}  ph: {sensor_data_processor.ph}  yw: {sensor_data_processor.yw}",
                         f"t: {sensor_data_processor.t}  p: {sensor_data_processor.p}  h: {sensor_data_processor.h}",
                         f"bp: {sensor_data_processor.bp}  ba: {sensor_data_processor.ba}  dt: {sensor_data_processor.dt}",
                         f"la: {sensor_data_processor.la}  lo: {sensor_data_processor.lo}  "
                         f"gs: {sensor_data_processor.gs}  al: {sensor_data_processor.al}  sn: {sensor_data_processor.sn}" ,
                         f"tq: {sensor_data_processor.tq}  cd: {sensor_data_processor.cd}   pr: {sensor_data_processor.pr}"]   
             }



        cached_lines = overlay_dict.get(overlay_mode, ["Invalid overlay mode"])

        

    
    # Periodically update overlay
    def apply_overlay(request):
        global frame_counter, cached_lines

        frame_counter += 1
        if not cached_lines or frame_counter % overlay_update_interval == 0:
            update_overlay_lines()
       
        with MappedArray(request, "main") as m:
            frame = m.array
            # Text settings
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 1.7
            thickness = 3
            line_height = 36
            padding = 20

            frame_h, frame_w = frame.shape[:2]

            text_sizes = [cv2.getTextSize(line, font, scale, thickness)[0] for line in cached_lines]
            max_width = max(w for w, h in text_sizes) if text_sizes else 0
            box_w = max_width + 80
            box_h = len(cached_lines) * line_height + 40
            box_x = max((frame_w - box_w) // 2, 0)
            box_y = max(frame_h - box_h - padding, 0)

            # solid black background
            cv2.rectangle(
                frame,
                (box_x, box_y),
                (box_x + box_w, box_y + box_h),
                (0, 0, 0),
                cv2.FILLED
            )
            # text
            for i, line in enumerate(cached_lines):
                text_width, _ = cv2.getTextSize(line, font, scale, thickness)[0]
                text_x = max((frame_w - text_width) // 2, 0)
                text_y = box_y + 50 + i * line_height
                cv2.putText(frame, line, (text_x, text_y), font, scale, (255, 255, 255), thickness)
          

  

         
    

# saving overlay to recordings causes video lag over time
    picam2.pre_callback = apply_overlay


   
    #///////////////////////////////////MAIN////////////////////////////////////
    #///////////////////////////////////////////////////////////////////////////


    def moving_average(a, n):
        ret = sum(a)
        return ret/n
    

        

    while True:
        # limit while loop frequency with time.sleep()
        time.sleep(0.001)
        millis = float(time.perf_counter())*1000
        
        #Other calculations
        #total speed in kph
        sensor_data_processor.ts = round((float(sensor_data_processor.c)*1.434866)*(60/1000),5)
        
        # Read from pedal if pipe is available
        if conn1.poll():
            pedals_data = conn1.recv()
            sensor_data_processor.pr = pedals_data[0]
            sensor_data_processor.cd = pedals_data[1]
            sensor_data_processor.tq = pedals_data[2]

        if (millis - time_last >=distance_calculation_interval): #Calculate distance in m from speed
            calculated_times += 1
            distance_traveled=distance_traveled+(sensor_data_processor.ts/3.6666)*((millis-time_last)/1000)
            time_last=millis
            sensor_data_processor.dt = int(distance_traveled)
            
        
        if GPIO.input(10) == GPIO.LOW and powerstate == False :
            if not file_open:
                file_open = True
            time.sleep(2)
            debounce = True
            debounce2 = True
            #/////////////////////////////FILE SETUP/////////////////////////////////////////////
            #/////////////////////////////////////////////////////////////////////////////////
            f=open(BASE_PATH + 'Saves/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
            file_open = True
            video_filename = BASE_PATH + 'Camera Videos/Vid_ ' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.h264'
            print("Starting Camera back up")
            update_overlay_lines()
            picam2.start_preview(Preview.QTGL, x=0, y=0, width = 1024, height = 600)
            picam2.start()
            # picam2.start_preview(Preview.QTGL, x=0, y=0, width = 1024, height = 600)
            # wait to initialize camera
            time.sleep(1)
            # record camera
            picam2.start_recording(encoder, video_filename)
            
            writer = csv.writer(f)
            #/////////////////////////Starting Camera and time/////////////////////////////// 
            #///////////////////////////////////////////////////////////////////////////////
            # (No annotation support here)
            start = dt.datetime.now()
            millis_start = time.perf_counter()*1000
            powerstate = True
            
        if GPIO.input(10) == GPIO.LOW:
            debounce = True
            debounce2 = True

            
            if (millis >(20*printed_times+time_start)):
                printed_times += 1
                line_count += 1
                auto_port_count += 1
                transmit_count += 1

                # Call to update overlay
               
                
                # run any calculations before this function is called
                if (run_once == True):
                   data_stream = sensor_data_processor.expected_data
                   data_stream.insert(0,  str("Time Stamp"))
                   writer.writerow(data_stream)
                   run_once = False
                
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
                    
            
        if GPIO.input(10) == GPIO.HIGH and powerstate == True and debounce == True:
            #/////DEBOUNCE IN CASE/////#
            time.sleep(1)
            debounce = False
            
            #/////////////////////////SAVING AND CLOSING FILES/////////////////////////////////
            #/////////////////////////////////////////////////////////////////////////////////
        if GPIO.input(10) == GPIO.HIGH and powerstate == True and debounce == False:
            time.sleep(1)
            print("Stopping camera and data...")
            end_time = time.time()
            f.close()
            file_open = False
            try:
                picam2.stop_preview()
            except RuntimeError:
                pass
            picam2.stop_recording()
            print("Stopped Camera...")
            powerstate = False
            ports_incomplete = True
            distance_traveled=0
            sensor_data_processor.pr = 0
            sensor_data_processor.cd = 0
            sensor_data_processor.tq = 0
            
            print("Stopped transmitting data...")
            
            
        if GPIO.input(10) == GPIO.HIGH and powerstate == False and debounce == False:
            debounce2 = True
        
        if GPIO.input(12) == GPIO.HIGH and GPIO.input(10) == GPIO.HIGH and powerstate == False and debounce2 == True:
            #/////DEBOUNCE IN CASE/////#
            time.sleep(2)
            debounce2 = False
            
        if GPIO.input(12) == GPIO.HIGH and GPIO.input(10) == GPIO.HIGH and powerstate == False and debounce2 == False:
            print("---------------------------------------------------------------------")
            print("Starting system shutdown process...")
            power_process.join(1)
            power_process.terminate()
            print("System shut down complete")
            sys.exit()
            

if __name__ == '__main__':
    system()

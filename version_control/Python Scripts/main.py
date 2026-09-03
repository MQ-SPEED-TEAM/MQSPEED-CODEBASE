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
from microcontroller_readings import SensorDataProcessor
import os
from multiprocessing import Process,Pipe,set_start_method
import pedal_readings as prd
from picamera2.utils import Transform
import cv2
import numpy as np
from picamera2.outputs import FfmpegOutput
import sys
from collections import deque




# Global overlay cache
cached_lines = []
frame_counter = 0
overlay_update_interval = 20  # Update overlay every 20 frames



#Switch/Button GPIO Pins
# (Pin IDs based on Pi5 GPIO PinOut)
# Red = GPIO 1 (3V3 power)
# Yellow = GPIO 15
# Black = GPIO 18

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

    
    
    #//////////////////////////WHEEL CIRCUMFERENCE CONSTANT/////////////////////////
    #/////////////////////////////// OLD WHEELS -----> 1.434m /////////////////////
    #//////////////////////////////  NEW WHEELS -----> 1.531m /////////////////////
    WHEEL_CIRCUMFERENCE = 1.531
    #WHEEL_CIRCUMFERENCE = 1.434
    #///////////////////////////////////////////////////////////////////////////////

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
    power_history = deque()
    bVma = 16.6 #battery voltage max
    bVmi = 13.0 #battery voltage min
    bVr = bVma - bVmi  #battery voltage range

    crank_teeth = 90 

    crank_rpm_history = deque(maxlen = 10)
    cassette_rpm_history = deque(maxlen = 10)

    

    cassette_teeth = {1: 32, 2: 28, 3: 25, 4: 22, 5: 20, 6: 17}

    gear_ratio_tolerance = 0.08
    gear_confirmation_count = 5
    gear_match_count = 0 
    last_selected_gear = 0
    shift_confirmed = False

   

    

 


    





    
    

     # setup function
    sensor_data_processor = SensorDataProcessor()
    sensor_data_processor.detected_gear = 0
    sensor_data_processor.shift_confirmed = False
    sensor_data_processor.gear_error_percent = 100.0
    sensor_data_processor.cr_avg = 0      
    sensor_data_processor.s_avg = 0

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
    
    
     #Setup camera settings
    encoder = H264Encoder()
    picam2 = Picamera2()
    video_config = picam2.create_video_configuration(main={"format": "XRGB8888", "size": (1640, 1232)},controls={"FrameRate": 30}, transform=Transform(rotation=0))
    picam2.configure(video_config)
    print("Starting camera preview")


    
    # Batteries percentage calculations
    bpPr = round((sensor_data_processor.bp - bVmi) / bVr * 100) # Main battery percentage
    baPr = round((sensor_data_processor.ba - bVmi) / bVr * 100) # Backup screen battery percentage
    bgPr = round((sensor_data_processor.bg - bVmi) / bVr * 100) # Gear battery percentage



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
                         f"cr: {sensor_data_processor.cr}  s: {sensor_data_processor.s}  ts: {sensor_data_processor.ts}  ",
                         f"dg: {sensor_data_processor.detected_gear} sc: {sensor_data_processor.shift_confirmed}  ",
                         f"cr_avg: {sensor_data_processor.cr_avg:.1f}  s_avg: {sensor_data_processor.s_avg:.1f}",
                         f"sa: {sensor_data_processor.sa}  g: {sensor_data_processor.g}  bg: {sensor_data_processor.bg}",
                         f"rl: {sensor_data_processor.rl}  ph: {sensor_data_processor.ph}  yw: {sensor_data_processor.yw}",
                         f"ax: {sensor_data_processor.ax}  ay: {sensor_data_processor.ay}  az: {sensor_data_processor.az}",
                         f"mx: {sensor_data_processor.mx}  my: {sensor_data_processor.my}  mz: {sensor_data_processor.mz}",
                         f"t: {sensor_data_processor.t}  p: {sensor_data_processor.p}  h: {sensor_data_processor.h}",
                         f"bp: {sensor_data_processor.bp}  ba: {sensor_data_processor.ba}  dt: {sensor_data_processor.dt}",
                         f"la: {sensor_data_processor.la}  lo: {sensor_data_processor.lo}  ",
                         f"gs: {sensor_data_processor.gs}  al: {sensor_data_processor.al}  sn: {sensor_data_processor.sn}" ,
                         f"tq: {sensor_data_processor.tq}  cd: {sensor_data_processor.cd}   pr: {sensor_data_processor.pr}"],   
            
             "imu": [f"rl: {sensor_data_processor.rl}  ph: {sensor_data_processor.ph}  yw: {sensor_data_processor.yw}",
                         f"qw: {sensor_data_processor.qw}  qx: {sensor_data_processor.qx}  qy: {sensor_data_processor.qy}  qz: {sensor_data_processor.qz}",
                         f"ax: {sensor_data_processor.ax}  ay: {sensor_data_processor.ay}  az: {sensor_data_processor.az}",
                         f"mx: {sensor_data_processor.mx}  my: {sensor_data_processor.my}  mz: {sensor_data_processor.mz}",
                         f"gx: {sensor_data_processor.gx}  gy: {sensor_data_processor.gy}  gz: {sensor_data_processor.gz}"],     
        }



        cached_lines = overlay_dict.get(overlay_mode, ["Invalid overlay mode"])
        
        
    #/////////////// Colour Gradient Function ////////////////    
    # ratio = actual_power / target_power
        
        #clamp ratio into 0 to 1.2
       # ratio = max(0.0, min(ratio, 1.2))
        
        #BGR colours
        #low = np.array([255,0,0])		# blue
        #mid = np.array([0,255,0])		# purple
        #high = np.array([0,0,255])		# red
#         
#         if ratio < 0.95:
#             t = (ratio - 0.7) / 0.7
#             colour = (1-t) * low + t * mid
#             
#         elif ratio > 1.05:
#             t = (ratio - 0.7) / 0.7
#             colour = (1 - t) * mid + t * high
#             
#         else:
#             colour = mid 
#         return tuple(int(c) for c in colour)
# 


    power_profile = [
        (500, 100),
        (1000, 300),
        (1500, 400),
        (2000, 250),
    ]

    def get_power_target(distance):
        for limit, power in power_profile:
            if distance < limit:
                return power
        return power_profile[-1][1]



    

    
    # Applying Overlay 
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
            line_height = 50
            padding = 20

            frame_h, frame_w = frame.shape[:2]

            text_sizes = [cv2.getTextSize(line, font, scale, thickness)[0] for line in cached_lines]
            max_width = max(w for w, h in text_sizes) if text_sizes else 0
            box_w = max_width + 80
            box_h = len(cached_lines) * line_height + 40
            box_x = max((frame_w - box_w) // 2, 0)
            box_y = max(frame_h - box_h - padding, 0) + 20

            # solid black background
            cv2.rectangle(
                frame,
                (box_x, box_y),
                (box_x + box_w, box_y + box_h),
                (0, 0, 0),
                cv2.FILLED
            )


            #//////////// Power Bar Graphic ////////////

            #Bar size and position
            bar_height = 195 
            bar_width = 50
            

            bar_max_power = 1000
            actual_power = max(0, sensor_data_processor.pr_avg)
            target_power = max(1, get_power_target(sensor_data_processor.dt))
            bar_scale_max = target_power * 2.2
            fill_ratio = min(actual_power / bar_scale_max, 1.0)
            fill_height = int(fill_ratio * bar_height)
            #bar_colour = get_gradient_colour(actual_power, target_power)
            
            #////////// Colour of Meter /////////
            if target_power + 1 < actual_power:
                bar_colour = (0,0,255)
                
            elif target_power - 1 > actual_power:
                bar_colour = (255,0,0)
                
            else:
                bar_colour = (0,255,0)

           


           


            bar_x = frame_w // 2 - 75
            bar_y = frame_h // 2 + 330 

            
            


            cv2.rectangle(
                frame,
                (bar_x, bar_y ),
                (bar_x + bar_width, bar_y + bar_height   ),
                (255, 255, 255),
                2
            )

            cv2.rectangle(
                frame,
                (bar_x, bar_y + bar_height - fill_height ),
                (bar_x + bar_width, bar_y + bar_height),
                bar_colour,
                cv2.FILLED
            )
            
            cv2.line(
                frame,
                (bar_x, bar_y + 150),
                (bar_x + bar_width, bar_y + 150 ),
                (255,255,255),
                4
            )
            cv2.line(
                frame,
                (bar_x, bar_y + 70  ),
                (bar_x + bar_width, bar_y + 70 ),
                (255,255,255),
                4
            )


            # triangle at top of bar
            triangle_height = 52
            triangle_half_width = 60

            triangle_points = np.array([
                [bar_x + bar_width // 2, bar_y - triangle_height],
                [bar_x + bar_width // 2 - triangle_half_width, bar_y],
                [bar_x + bar_width // 2 + triangle_half_width, bar_y],
            ], dtype=np.int32)

            cv2.fillPoly(frame, [triangle_points], bar_colour)
            
            cv2.putText(
                frame,
                f"{target_power}",
                (bar_x  , bar_y ),
                font,
                0.8,
                (255, 255, 255),
                2
            )
            
            
                
            
    

  
            # text
            for i, line in enumerate(cached_lines):
                text_width, _ = cv2.getTextSize(line, font, scale, thickness)[0]
                text_x = max((frame_w - text_width) // 2, 0)
                text_y = box_y + 50 + i * line_height
                cv2.putText(frame, line, (text_x, text_y), font, scale, (255, 255, 255), thickness)
                
                
                
                
                
            if (
                sensor_data_processor.gear_home_complete
                and time.time() - sensor_data_processor.gear_home_message_time < 3
                ):
                
                message = "GEAR HOMING COMPLETE"
                
                popup_scale= 1.5
                popup_thickness = 4
                
                
                text_width, text_height = cv2.getTextSize(
                    message,
                    font,
                    popup_scale,
                    popup_thickness
                    )[0]
                
                popup_padding_x = 40
                popup_padding_y =30
                
                popup_x = (frame_w - text_width) // 2
                popup_y = frame_h //3
                
                
                
                # Black popup background
                
                
                cv2.rectangle(
                    frame,
                    (
                        popup_x - popup_padding_x,
                        popup_y - text_height - popup_padding_y,
                    ),
                    (
                        popup_x + text_width +popup_padding_x,
                        popup_y + popup_padding_y
                    ),
                    (0,0,0),
                    cv2.FILLED
                )
                
                #White border
                cv2.rectangle(
                    frame,
                    (
                        popup_x - popup_padding_x,
                        popup_y - text_height - popup_padding_y
                    ),
                    (
                        popup_x + text_width + popup_padding_x,
                        popup_y + popup_padding_y
                        
                    ),
                    (255, 255, 255),
                    4
                )
                
                
                #Popup text
                cv2.putText(
                    frame,
                    message,
                    (popup_x, popup_y),
                    font,
                    popup_scale,
                    (255, 255, 255),
                    popup_thickness
                )
                
            elif sensor_data_processor.gear_home_complete:
                sensor_data_processor.gear_home_complete = False
                    
                
                


      
          

  

         
    


    picam2.pre_callback = apply_overlay


   
    #///////////////////////////////////MAIN////////////////////////////////////
    #///////////////////////////////////////////////////////////////////////////


    def moving_average(a, n):
        ret = sum(a)
        return ret/n


    def detect_actual_gear(crank_cadence_rpm, cassette_rpm):
    
  

        # Avoid detecting gears when the drivetrain is barely moving.
        if crank_cadence_rpm < 20 or cassette_rpm < 20:
            return 0, 100.0

        measured_ratio = cassette_rpm / crank_cadence_rpm

        closest_gear = 0
        smallest_error = float("inf")

        for gear, rear_teeth in cassette_teeth.items():
            expected_ratio = crank_teeth / rear_teeth

            error = abs(measured_ratio - expected_ratio) / expected_ratio

            if error < smallest_error:
                smallest_error = error
                closest_gear = gear

        error_percent = smallest_error * 100

    # Reject the result if it is not close enough to any known gear.
        if smallest_error > gear_ratio_tolerance:
            return 0, error_percent

        return closest_gear, error_percent

        

    while True:
        # limit while loop frequency with time.sleep()
        time.sleep(0.001)
        millis = float(time.perf_counter())*1000
        
        #Other calculations
        #total speed in kph
        sensor_data_processor.ts = round((float(sensor_data_processor.c) *1.531)*(60/1000),5)
        
        # Read from pedal if pipe is available
        if conn1.poll():
            pedals_data = conn1.recv()
            sensor_data_processor.pr = pedals_data[0]
            sensor_data_processor.cd = pedals_data[1]
            sensor_data_processor.tq = pedals_data[2]
            
            now = time.time()
            
            #Store the (timestamp, power)
            power_history.append((now, sensor_data_processor.pr))
            
            #Keep only last 3 seconds
            while power_history and power_history[0][0] < now -3:
                power_history.popleft()
                
            # Rolling Average 3 Seconds
            sensor_data_processor.pr_avg = (
                sum(p for _, p in power_history) / len(power_history)
            )

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
            f=open(BASE_PATH + 'CSV/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
            file_open = True
            video_filename = BASE_PATH + 'VIDEO/Vid_ ' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.h264'
            print("Starting Camera back up")
            update_overlay_lines()
            picam2.start_preview(Preview.QTGL, x=0, y=0, width=1024, height=600)
            for window in QApplication.topLevelWidgets():
            window.showFullScreen()

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
                
                # Raw RPM readings
                crank_rpm_raw = float(sensor_data_processor.cr)
                cassette_rpm_raw = float(sensor_data_processor.s)

                # Add readings to smoothing buffers
                if crank_rpm_raw > 0:
                    crank_rpm_history.append(crank_rpm_raw)

                if cassette_rpm_raw > 0:
                    cassette_rpm_history.append(cassette_rpm_raw)

                # Average recent readings
                if crank_rpm_history:
                    crank_cadence_rpm = sum(crank_rpm_history) / len(crank_rpm_history)
                else:
                    crank_cadence_rpm = 0

                if cassette_rpm_history:
                    cassette_rpm = sum(cassette_rpm_history) / len(cassette_rpm_history)
                else:
                    cassette_rpm = 0

                # Storing avg the rpm for the overlay
                sensor_data_processor.cr_avg = crank_cadence_rpm
                sensor_data_processor.s_avg = cassette_rpm

                # Detect gear from smoothed RPM
                detected_gear, gear_error_percent = detect_actual_gear(
                crank_cadence_rpm,
                cassette_rpm
                )


                
                selected_gear = int(round(sensor_data_processor.g))

                if selected_gear != last_selected_gear:
                    last_selected_gear = selected_gear
                    gear_match_count = 0
                    shift_confirmed = False

                if detected_gear == selected_gear and detected_gear != 0:
                    gear_match_count += 1

                    if gear_match_count >= gear_confirmation_count:
                        gear_match_count = gear_confirmation_count
                        shift_confirmed = True


                else:
                    gear_match_count = max(gear_match_count - 1, 0)
                    if gear_match_count == 0:
                        shift_confirmed = False


                sensor_data_processor.detected_gear = detected_gear
                sensor_data_processor.shift_confirmed = shift_confirmed
                sensor_data_processor.gear_error_percent = gear_error_percent
                

              
                
                data_stream.insert(0, str(datetime.now().strftime('%H_%M_%S_%f'))[:-3])
                
                writer.writerow(data_stream)
                
                if transmit_count >= 40:
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
            

if _name_ == '_main_':
    system()



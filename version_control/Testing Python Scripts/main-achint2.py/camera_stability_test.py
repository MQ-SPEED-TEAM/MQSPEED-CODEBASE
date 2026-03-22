from picamera2 import Picamera2, Preview, MappedArray
from picamera2.encoders import H264Encoder
from picamera2.utils import Transform
import time
import RPi.GPIO as GPIO
import csv
import datetime as dt
from datetime import datetime


# Setup camera settings
encoder = H264Encoder()
picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"format": "XRGB8888", "size": (1640, 1232)},controls={"FrameRate": 30}, transform=Transform(rotation=0))
picam2.configure(video_config)
print("Starting camera preview")
powerstate = False
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
debounce = True
debounce2 = True
BASE_PATH = '/home/mqspeed/Desktop/'


while True:
    time.sleep(0.001)
    if GPIO.input(10) == GPIO.LOW and powerstate == False :
        debounce = True
        debounce2 = True
#         f=open(BASE_PATH + 'Saves/Test_' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.csv', 'w')
        file_open = True
#         video_filename = BASE_PATH + 'Camera Videos/Vid_ ' + str(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + '.h264'
        print("Starting Camera back up")
        picam2.start_preview(Preview.QTGL, x=0, y=0, width = 1024, height = 600)
        picam2.start()
#         time.sleep(1)
        # record camera
#         picam2.start_recording(encoder, video_filename)
        powerstate = True
    
    if GPIO.input(10) == GPIO.LOW:
        debounce = True
        debounce2 = True
        
    if GPIO.input(10) == GPIO.HIGH and powerstate == True and debounce == True:
            #/////DEBOUNCE IN CASE/////#
            time.sleep(1)
            debounce = False
    
    if GPIO.input(10) == GPIO.HIGH and powerstate == True and debounce == False:
            time.sleep(1)
            print("Stopping camera and data...")
            try:
                picam2.stop_preview()
            except RuntimeError:
                pass
            picam2.stop_recording()
            print("Stopped Camera...")
            powerstate = False
    
    if GPIO.input(10) == GPIO.HIGH and powerstate == False and debounce == False:
            debounce2 = True
        
    if GPIO.input(12) == GPIO.HIGH and GPIO.input(10) == GPIO.HIGH and powerstate == False and debounce2 == True:
         #/////DEBOUNCE IN CASE/////#
        time.sleep(2)
        debounce2 = False
            
    if GPIO.input(12) == GPIO.HIGH and GPIO.input(10) == GPIO.HIGH and powerstate == False and debounce2 == False:
        print("System shut down complete")
        sys.exit()

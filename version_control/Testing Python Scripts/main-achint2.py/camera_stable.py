from picamera2 import Picamera2, Preview, MappedArray
from picamera2.encoders import H264Encoder
from picamera2.utils import Transform


# Setup camera settings
encoder = H264Encoder()
picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"format": "XRGB8888", "size": (1640, 1232)},controls={"FrameRate": 30}, transform=Transform(rotation=0))
picam2.configure(video_config)
print("Starting camera preview")
picam2.start_preview(Preview.QTGL, x=0, y=0, width = 1024, height = 600)
picam2.start()
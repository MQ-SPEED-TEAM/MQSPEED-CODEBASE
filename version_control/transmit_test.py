import serial
import time


hatch = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
time.sleep(1)

for i in range(1000):
    hatch.write(b"{i}")
    hatch.write(b"\n")
    time.sleep(0.5)
    print(f"number {i} sent")

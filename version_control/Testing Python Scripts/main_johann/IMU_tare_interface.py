import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

def main():
    print("Connecting to ESP32...")
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
        ser.reset_input_buffer()
        time.sleep(2)  # allow ESP32 to reset
    except:
        ser = None
        print("ESP NOT FOUND, DOUBLE CHECK PORT ASSIGNMENT")
        

    print("Connected. Type commands:")
    print("  tare     → apply imu.tareNow()")
    print("  persist  → apply imu.saveTare()")
    print("  clear    → apply imu.clearTare()")
    print("  orientation    → read current roll, pitch, and yaw")
    print("  quit     → exit program")

    while True:
        cmd = input("> ").strip().lower()

        if cmd == "quit":
            print("Exiting.")
            break

        if cmd in ["tare", "persist", "clear", "orientation"]:
            ser.write((cmd + "\n").encode())
            ser.flush()

            # read response
            time.sleep(0.3)
            while ser.in_waiting:
                print("ESP32:", ser.readline().decode().strip())

        else:
            print("Unknown command.")

    ser.close()

if __name__ == "__main__":
    main()

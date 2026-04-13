import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

def main():
    print("Connecting to ESP32...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # allow ESP32 to reset

    print("Connected. Type commands:")
    print("  tare     → apply imu.tareNow()")
    print("  persist  → apply imu.saveTare()")
    print("  clear    → apply imu.clearTare()")
    print("  quit     → exit program")

    while True:
        cmd = input("> ").strip().lower()

        if cmd == "quit":
            print("Exiting.")
            break

        if cmd in ["tare", "persist", "clear"]:
            ser.write((cmd + "\n").encode())
            ser.flush()

            # read response
            time.sleep(0.1)
            while ser.in_waiting:
                print("ESP32:", ser.readline().decode().strip())

        else:
            print("Unknown command.")

    ser.close()

if __name__ == "__main__":
    main()

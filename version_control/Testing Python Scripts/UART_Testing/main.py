import serial
import time
from microcontroller_readings import SensorDataProcessor
from SafeUART import SafeUART


    
if __name__ == '__main__':
    # uart = SafeUART(serial.Serial(port='/dev/ttyAMA3', baudrate=115200, 
    #                                        timeout=1, parity=serial.PARITY_NONE,
    #                                        stopbits=serial.STOPBITS_ONE,
    #                                        bytesize=serial.EIGHTBITS))
    # uart.serial.reset_input_buffer()
    # time.sleep(1)

    ports_incomplete = True
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

    # print("ports complete status:")
    # print(not ports_incomplete)
    # uart.send_data(b'Hello')

    # length, data = uart.receive_data(128)
    # if length > 0:
    #     print(data)
    # uart.close()
       
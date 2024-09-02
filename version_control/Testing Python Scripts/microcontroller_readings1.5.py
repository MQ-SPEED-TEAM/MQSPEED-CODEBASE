import serial
import ast
import time
import chardet

import csv
from datetime import datetime


test_data = {'c': 0, 'l': 0, 'r': 0, 'cr': 0, 's': 0, 'ts': 0, 'sa': 0, 'g': 1, 'bg': 266, 'ax': -1.84, 'ay': 6.27, 'az': -7.59, 'vx': -20.55, 'vy': 4.65, 'vz': -97.05, 't': 24.92, 'p': 102189.0, 'h': 53.03, 'bp': 15.26, 'ba': 0.54, 'data': 0}
x = 5
class SensorDataProcessor:
    """Takes data from all the sensors and packages them for either data logging, 
    screen display, or to send back to microcontrollers"""

    def __init__(self) -> None:
        # initialize all expected data as class instance attributes
        self.c = 0
        self.l = 0
        self.r = 0
        self.cr = 0
        self.s = 0
        self.ts = 0
        self.sa = 0
        self.g = 0
        self.bg = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.t = 0
        self.p = 0
        self.h = 0
        self.bp = 0
        self.ba = 0
        self.dt = 0
        self.la = 0
        self.lo = 0
        self.df = 0
        self.last_log = 0
        self.active_ports = 0

        # Initialize expected data as a list for comparison in update_attributes function
        self.expected_data = ["c","l","r","cr",
                              "s","ts","sa","g","bg","ax",
                              "ay","az","vx",
                              "vy","vz","t","h",
                              "p","bp","ba","la","lo","df","dt"]
        # Initialize port instances as class attribute
        self.esphatch = 0
        self.espbike = 0
        self.espgear = 0
        self.defects = 0 # Variable for testing serial reading consistency when debugging
        self.expected_ports = 3 # Sets the number of ports needed to be detected before check_port_complete function returns True
        
        # legend in order : central wheel=c, left wheel=l, right wheel=r, crank=cr, shaft=s, total speed=ts, gear set=g,
        # steering angle=sa, acceleration x=ax, acceleration y=ay, acceleration z=az, angular velocity x=vx, angular velocity y=vy
        # angular velocity z=vz, temperature=t, humidity=h, air pressure=p, latitude=la, longitude=lo, distance to finish=df, distance traveled = dt
    
    def bytestream_read(self, port=None, ending_line=b"\n"):
        """Takes serial port instance and ending byte as arguments. Returns list of decoded byte characters starting from
        and including possible starting bytes up to but not including ending byte"""
        current_array = 0
        byte_array_temp = []
        possible_start_bytes = [b"h", b"b", b"g"]
        start_byte_found = False
        
        
        if port == None or port == 0 or port == "None":
            return ("port unavailable")
        if port != None:
            if not port.is_open:
                port.open()
        
        # Check for port availability and start byte
        while True:
            if not start_byte_found:
                try:
                    byte_read = port.read()
                except serial.serialutil.SerialException:
                    print("SerialException, port unavailable")
                    return ("port unavailable")
                if byte_read in possible_start_bytes:
                    start_byte_found = True
                    byte_char = byte_read.decode() 
                    byte_array_temp.append(byte_char)
                    break
        # Read incoming bytes
        if start_byte_found:
            while current_array < 1:
                    try:
                        byte_read = port.read()  #  Read individual byte from serial port
                    except serial.serialutil.SerialException:
                        return ("port unavailable")
                    if byte_read: #if not empty
                        byte_char = byte_read.decode()  # Decode bytes to string
                        byte_array_temp.append(byte_char) # Add decoded byte to temporary array
                        if byte_read == ending_line:
                            current_array += 1
                            del byte_array_temp[-2:] # remove newline character
                            start_byte_found = False
            
            port.reset_input_buffer()
            return byte_array_temp
    
    def auto_port(self):
        """Initialize and automatically detect and label port devices"""
        # Initialize ports
        try: 
            esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
            esp_1.reset_input_buffer()
            time.sleep(1)
        except:
            esp_1 = None
            print("USB0 DEACTIVATED")
        
        try:
            esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
            esp_2.reset_input_buffer()
            time.sleep(1)
        except:
            esp_2 = None
            print("USB1 DEACTIVATED")
        
        try:
            esp_3 = serial.Serial('/dev/ttyUSB2', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
            esp_3.reset_input_buffer()
            time.sleep(1)
        except:
            esp_3 = None
            print("USB2 DEACTIVATED")
        
        try:
            esp_4 = serial.Serial('/dev/ttyUSB3', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
            esp_4.reset_input_buffer()
            print(esp_4)
            time.sleep(1)
        except:
            esp_4 = None
            print("USB3 DEACTIVATED")
        
        # Label ports depending on valid start bytes found in serial port byte stream
        array_checks = 10
        byte_array_temp =[]
        
        current_array = 0
        if esp_1 != None:
            while current_array <= array_checks:
                byte_read = esp_1.read()  # Read bytes from serial port
                if byte_read: #if not empty
                    try:
                        byte_char = byte_read.decode()  # Decode bytes to string
                    except UnicodeDecodeError:
                        byte_char = None
                        byte_read = None
                        print("UnicodeDecodeError averted")
                    byte_array_temp.append(byte_char) # Add decoded byte to temporary array
                    if byte_read == b"\n" and "h" in byte_array_temp:
                        print("hatch connected")
                        self.esphatch = esp_1
                    if byte_read == b"\n" and "b" in byte_array_temp:
                        print("bike connected")
                        self.espbike = esp_1
                    if byte_read == b"\n" and "g" in byte_array_temp:
                        print("gear connected")
                        self.espgear = esp_1
                    if byte_read == b"\n":
                        byte_array_temp =[]
                        current_array += 1
                        try:
                            esp_1.reset_input_buffer()
                        except AttributeError:
                            esp_1 = None
                
                
        current_array = 0
        if esp_2 != None:
            while current_array <= array_checks:
                byte_read = esp_2.read()  # Read bytes from serial port
                if byte_read: #if not empty
                    try:
                        byte_char = byte_read.decode()  # Decode bytes to string
                    except UnicodeDecodeError:
                        byte_char = None
                        byte_read = None
                        print("UnicodeDecodeError averted")
                    byte_array_temp.append(byte_char) # Add decoded byte to temporary array
                    if byte_read == b"\n" and "h" in byte_array_temp:
                        print("hatch connected")
                        self.esphatch = esp_2
                    if byte_read == b"\n" and "b" in byte_array_temp:
                        print("bike connected")
                        self.espbike = esp_2
                    if byte_read == b"\n" and "g" in byte_array_temp:
                        print("gear connected")
                        self.espgear = esp_2
                    if byte_read == b"\n":
                        byte_array_temp =[]
                        current_array += 1
                        try:
                            esp_2.reset_input_buffer()
                        except AttributeError:
                            esp_2 = None
        
        current_array = 0
        if esp_3 != None:
            while current_array <= array_checks:
                byte_read = esp_3.read()  # Read bytes from serial port
                if byte_read: #if not empty
                    try:
                        byte_char = byte_read.decode()  # Decode bytes to string
                    except UnicodeDecodeError:
                        byte_char = None
                        byte_read = None
                        print("UnicodeDecodeError averted")
                    byte_array_temp.append(byte_char) # Add decoded byte to temporary array
                    if byte_read == b"\n" and "h" in byte_array_temp:
                        print("hatch connected")
                        self.esphatch = esp_3
                    if byte_read == b"\n" and "b" in byte_array_temp:
                        print("bike connected")
                        self.espbike = esp_3
                    if byte_read == b"\n" and "g" in byte_array_temp:
                        print("gear connected")
                        self.espgear = esp_3
                    if byte_read == b"\n":
                        byte_array_temp =[]
                        current_array += 1
                        try:
                            esp_3.reset_input_buffer()
                        except AttributeError:
                            esp_3 = None
        
        current_array = 0
        if esp_4 != None:
            while current_array <= array_checks:
                byte_read = esp_4.read()  # Read bytes from serial port
                if byte_read: #if not empty
                    try:
                        byte_char = byte_read.decode()  # Decode bytes to string
                    except UnicodeDecodeError:
                        byte_char = None
                        byte_read = None
                        print("UnicodeDecodeError averted")
                    byte_array_temp.append(byte_char) # Add decoded byte to temporary array
                    if byte_read == b"\n" and "h" in byte_array_temp:
                        print("hatch connected")
                        self.esphatch = esp_4
                    if byte_read == b"\n" and "b" in byte_array_temp:
                        print("bike connected")
                        self.espbike = esp_4
                    if byte_read == b"\n" and "g" in byte_array_temp:
                        print("gear connected")
                        self.espgear = esp_4
                    if byte_read == b"\n":
                        byte_array_temp =[]
                        current_array += 1
                        try:
                            esp_4.reset_input_buffer()
                        except AttributeError:
                            esp_4 = None
        
                            
        # Print port status            
        print("USB0 read:")
        if esp_1 == None:
            print("Not Connected")
        else:
            print(esp_1)
        print("USB1 read:")
        if esp_2 == None:
            print("Not Connected")
        else:
            print(esp_2)
        print("USB2 read:")
        if esp_3 == None:
            print("Not Connected")
        else:
            print(esp_3)
        print("USB3 read:")
        if esp_4 == None:
            print("Not Connected")
        else:
            print(esp_4)
        
        print(''.join(self.bytestream_read(self.esphatch)))
        print(''.join(self.bytestream_read(self.espbike)))
        print(''.join(self.bytestream_read(self.espgear)))
        
        # Retrieve labels of working ports
        returned_ports = []
        if self.bytestream_read(self.esphatch) != "port unavailable":
            returned_ports.append("hatch")
        else:
            returned_ports.append("none")
        
        if self.bytestream_read(self.espbike) != "port unavailable":
            returned_ports.append("bike")
        else:
            returned_ports.append("none")
        
        if self.bytestream_read(self.espgear) != "port unavailable":
            returned_ports.append("gear")
        else:
            returned_ports.append("none")
        
        print("returned ports:")
        print(returned_ports)
            
        return returned_ports
            
            
    def check_port_complete(self, ports_returned = list):
        """Takes list of port labels as an argument. Returns boolean describing port device completeness."""
        # Check if all ports are actively in use
#         expected_ports = 3
        if self.active_ports < self.expected_ports:
            self.active_ports = 0
            for port in ports_returned:
                if 'none' not in port:
                    self.active_ports += 1
                    print("port detected as: ")
                    print(port)
                    
        # Return boolean
        if self.active_ports == self.expected_ports:
            print("all ports connected, terminating port check.")
            return True
        elif self.active_ports < self.expected_ports:
            print("not all ports connected, continuing port check.")
            return False
        else:
            print("more than " + str(self.expected_ports) + " serial connections established")
            return True
            
    
    def open_port_for_test(self):
        """Debugging method for testing port opening"""
        self.esphatch = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
        self.esphatch.reset_input_buffer()
        self.esphatch.reset_output_buffer()
        time.sleep(3)
        
    def raw_read(self):
        """Debugging method for seeing what the pi first sees when reading serial"""
        self.espgear = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
        array_tmp = []
        while True:
            byte_read = self.espgear.read()  # Read bytes from serial port
            if byte_read: #if not empty
                byte_char = byte_read.decode()  # Decode bytes to string
                array_tmp.append(byte_char) # Add decoded byte to temporary array
                if byte_read == b"\n":
                    print(''.join(array_tmp))  # Print the received line                       
                    if array_tmp[-1] != "\n" or len(array_tmp) < 24 :
                        self.defects += 1
                    self.espgear.reset_input_buffer()
                    array_tmp = []
                    break       

    def format_data(self):
        """This method reads the serials, compares serial values with expected dict keys,
        before matching serial values to dickt key labels then returning as a dictionary"""
        # Declare list of dict keys
        esp_bike_dictkeys = ["c","l","r","s","cr","sa"]
        esp_gear_dictkeys = ["g","bg"]
        esp_hatch_dictkeys = ["ax","ay","az","vx","vy","vz","t","h",
                              "p","bp","ba"]
        cal_data_dict = {"ts":self.ts, "dt":self.dt}
        
        # Match list of dict keys to serial port list of data points
        while True:
            
            
            esp_bike_raw =  ''.join(self.bytestream_read(self.espbike))           
            if esp_bike_raw == "port unavailable":
                esp_bike_dict = {"bike": "unavailable"}
            else:
                esp_bike_raw = esp_bike_raw[2:]
                esp_bike_list = list(esp_bike_raw.split(","))
                esp_bike_dict = {esp_bike_dictkeys[i]: float(esp_bike_list[i]) for i in range(len(esp_bike_dictkeys))}
            

            esp_gear_raw =  ''.join(self.bytestream_read(self.espgear))
            if esp_gear_raw == "port unavailable":
                esp_gear_dict =  {"gear": "unavailable"}

            else:
                esp_gear_raw = esp_gear_raw[2:]
                esp_gear_list = list(esp_gear_raw.split(","))
                try:
                    esp_gear_dict = {esp_gear_dictkeys[i]: float(esp_gear_list[i]) for i in range(len(esp_gear_dictkeys))}
                    if esp_gear_dict["bg"] < 14.0:
                        self.espgear = None
                except IndexError: # Error likely due to battery analog reading flickering
                    esp_gear_list.append('0')
                    esp_gear_dict = {esp_gear_dictkeys[i]: float(esp_gear_list[i]) for i in range(len(esp_gear_dictkeys))}
                except ValueError: #
                    esp_gear_dict = {"g":0,"bg":0}
            
            
            
            esp_hatch_raw =  ''.join(self.bytestream_read(self.esphatch))
            if esp_hatch_raw == "port unavailable":
                esp_hatch_dict =  {"hatch": "unavailable"}
            else:
                esp_hatch_raw = esp_hatch_raw[2:]
                esp_hatch_list = list(esp_hatch_raw.split(","))
                esp_hatch_dict = {esp_hatch_dictkeys[i]: float(esp_hatch_list[i]) for i in range(len(esp_hatch_dictkeys))}
            
            
            # Combine dictionaries
            data_dict = {**esp_bike_dict, **esp_gear_dict, **esp_hatch_dict, **cal_data_dict}
            
            
            try:
                if esp_bike_raw == "port unavailable":
                    break
                else:
                    pass
            except:
                print('fail')

            break

            
        return data_dict
    
    def update_attributes(self, data_as_dict):
        """This method updates the class instance attributes based on the formatted dictionary as an argument.
        Additionally, it registers the attribute as '0' if the sensor fails to send it
        to the pi by comparing the argument dictionary to the expected_data class atribute dictionary."""
        # Initialize all data recieved
        available_data = []
        for key in data_as_dict:
            setattr(self, key, data_as_dict[key])
            available_data.append(key)
        # Replace all class attributes not found in data recieved with "0"
        unavailable_data = list(set(available_data).symmetric_difference(set(self.expected_data)))
        if len(unavailable_data) > 0:
            unavailable_data_as_dict= dict.fromkeys(unavailable_data, 0)
            for key in unavailable_data_as_dict:
                setattr(self, key, unavailable_data_as_dict[key])
    
    def process(self):
        """This method runs the update_attributes and format_data methods in the appropriate order,
        then returns a list of the current class instance attributes related to sensor data values."""
        self.update_attributes(self.format_data())
        current_data = dict(self.__dict__)
        # Remove all uneccesary class attributes before returning final list of data values
        current_data.pop("expected_data")
        current_data.pop("esphatch")
        current_data.pop("espbike")
        current_data.pop("espgear")
        current_data.pop("defects")
        current_data.pop("last_log")
        current_data.pop("la")
        current_data.pop("lo")
        current_data.pop("df")
        current_data.pop("active_ports")
        current_data.pop("expected_ports")
        
#         print(current_data.values())
        return list(current_data.values())
    
    def transmit(self):
        """This method transmits the current class atrributes as a byte stream to the hatch esp."""
        current_data = dict(self.__dict__)
        # Remove all uneccesary class attributes before returning final list of data values
        current_data.pop("expected_data")
        current_data.pop("esphatch")
        current_data.pop("espbike")
        current_data.pop("espgear")
        current_data.pop("defects")
        current_data.pop("last_log")
        current_data.pop("la")
        current_data.pop("lo")
        current_data.pop("df")
        current_data.pop("active_ports")
        current_data.pop("expected_ports")
        
        # Format dictionary into byte stream
        current_data_line = str(list(current_data.values()))[1:-1]
        current_data_list = list(current_data.values())
        try:
            current_data_list = [float(i) for i in current_data_list]
        except ValueError: # Again likely to gearbox data flickering
            current_data_list = [i for i in current_data_list if i != '']
            current_data_list = [float(i) for i in current_data_list]
            print("gearbox gave too much data")
        current_data_line = str(current_data_list)[1:-1]
        current_data_string = f"{current_data_line}\n"
        current_data_bytes = str.encode(current_data_string)
        
        try:
            print(f"sending data")
            self.esphatch.write(current_data_bytes)
            print(f"data sent: {current_data_line}")
        except AttributeError:
            print(f"data packet not sent: {current_data_line}")
        except OSError:
            print(f"Hatch ESP disconnected, transmit error")
        return current_data_string











### DEBUGGING BOILER PLATE DONT MIND
sensor_processor = SensorDataProcessor()
# # bike_esp = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, xonxoff=False, rtscts=True, dsrdtr=True)
# 
# sensor_processor.check_port_complete(sensor_processor.auto_port())
# # sensor_processor.auto_port()
# 
# #Benchmark test functions
# 
# ##
count = 0
while count <= 1000:
    stopwatch_start = time.perf_counter_ns()
    sensor_processor.raw_read()
    sensor_processor.bytestream_read(bike_esp)
#     print(sensor_processor.process())
    print(sensor_processor.transmit())
    stopwatch_end = time.perf_counter_ns()
    print(stopwatch_end - stopwatch_start)
    count += 1
print("reading defects:")
print(sensor_processor.defects)
##

# writer.writerow([sensor_processor.defects])

# f.close()

# # while True:
#     sensor_processor.raw_read()




import serial
import ast
import time

test_data = {'c': 0, 'l': 0, 'r': 0, 'cr': 0, 's': 0, 'ts': 0, 'sa': 0, 'g': 1, 'bg': 266, 'ax': -1.84, 'ay': 6.27, 'az': -7.59, 'vx': -20.55, 'vy': 4.65, 'vz': -97.05, 't': 24.92, 'p': 102189.0, 'h': 53.03, 'bp': 15.26, 'ba': 0.54, 'data': 0}

class SensorDataProcessor:
    """Takes data from all the sensors and packages them for either data logging, 
    screen display, or to send back to microcontrollers"""

    def __init__(self) -> None:
        # set all expected data as class instance attributes
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
        self.la = 0
        self.lo = 0
        self.df = 0
        self.last_log = 0
        self.active_ports = 0

        # initialize expected data as a list for comparison
        self.expected_data = ["c","l","r","cr",
                              "s","ts","sa","g","bg","ax",
                              "ay","az","vx",
                              "vy","vz","t","h",
                              "p","bp","ba","la","lo","df"]
        self.esphatch = 0
        self.espbike = 0
        self.espgear = 0
        # legend in order : central wheel=c, left wheel=l, right wheel=r, crank=cr, shaft=s, total speed=ts, gear set=g,
        # steering angle=sa, acceleration x=ax, acceleration y=ay, acceleration z=az, angular velocity x=vx, angular velocity y=vy
        # angular velocity z=vz, temperature=t, humidity=h, air pressure=p, latitude=la, longitude=lo, distance to finish=df
        
    
        
            
    
    def auto_port(self):
        # initialize serial read and automatically label expected ports
        try: 
            esp_1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            esp_1.reset_input_buffer()
        except:
            esp_1 = "none"
        
        try:
            esp_2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            esp_2.reset_input_buffer()
        except:
            esp_2 = "none"
        
        try:
            esp_3 = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
            esp_3.reset_input_buffer()
        except:
            esp_3 = "none"
        
        try:
            esp_4 = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
            esp_4.reset_input_buffer()
        except:
            esp_4 = "none"
        
        # Check if data is complete, and if so, to sort ports
        while True:
            try:    
                esp_reading1 = esp_1.readline().decode('utf-8').rstrip()
            except:
                esp_reading1 = "'data': 'none'"
            try:
                esp_reading2 = esp_2.readline().decode('utf-8').rstrip()
            except:
                esp_reading2 = "'data': 'none'"
            try:
                esp_reading3 = esp_3.readline().decode('utf-8').rstrip()
            except:
                esp_reading3 = "'data': 'none'"
            try:
                esp_reading4 = esp_4.readline().decode('utf-8').rstrip()
            except:
                esp_reading4 = "'data': 'none'"
                
            reading_string = ''.join(("{",esp_reading1,",",esp_reading2,",",esp_reading3,",",esp_reading4,"}"))
            
            try:
                data_dict = ast.literal_eval(reading_string)
                print("USB0 read:")
                print(esp_reading1)
                print("USB1 read:")
                print(esp_reading2)
                print("USB2 read:")
                print(esp_reading3)
                print("USB3 read:")
                print(esp_reading4)

                            
                if "v" in esp_reading1:
                    self.esphatch = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
                elif "v" in esp_reading2:
                    self.esphatch = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
                elif "v" in esp_reading3:
                    self.esphatch = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
                elif "v" in esp_reading4:
                    self.esphatch = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
                else:
                    self.esphatch = "none"
                    print("hatch esp not available")
                
                if "c" in esp_reading1:
                    self.espbike = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
                elif "c" in esp_reading2:
                    self.espbike = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
                elif "c" in esp_reading3:
                    self.espbike = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
                elif "c" in esp_reading4:
                    self.espbike = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
                else:
                    self.espbike = "none"
                    print("bike esp not available")
                    
                if "g" in esp_reading1:
                    self.espgear = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
                elif "g" in esp_reading2:
                    self.espgear = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
                elif "g" in esp_reading3:
                    self.espgear = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
                elif "g" in esp_reading4:
                    self.espgear = serial.Serial('/dev/ttyUSB3', 115200, timeout=1)
                else:
                    self.espgear = "none"
                    print("gear esp not available")
                    
                return [esp_reading1,esp_reading2,esp_reading3,esp_reading4]
                    
                
                break
                            
            except SyntaxError:
                print("initial data format error: ")
                print(reading_string)
                continue
            
            except ValueError:
                print("initial value error due to malformed node or string")
                continue
            
    def check_port_complete(self, ports_returned = list):
        # check if all ports are actively in use
        expected_ports = 3
        if self.active_ports < expected_ports:
            self.active_ports = 0
            for port in ports_returned:
                if 'none' not in port:
                    self.active_ports += 1
                    print("port detected as: ")
                    print(port)
        if self.active_ports == expected_ports:
            print("all ports connected, terminating port check.")
            return True
        elif self.active_ports < expected_ports:
            print("not all ports connected, continuing port check.")
            return False
        else:
            print("more than 3 serial connections established")
            return True
            
    
    
    def raw_read(self):
        """debugging method for seeing what the pi first sees when reading serial"""
        print("esp_bike:")
        print(self.espbike.readline().decode('utf-8').rstrip())
        print("esp_hatch:")
        print(self.esphatch.readline().decode('utf-8').rstrip())
        print("esp_gear:")
        print(self.espgear.readline().decode('utf-8').rstrip())
        

    def format_data(self):
        """This method reads the serials and joins them together as a string,
        before returning as a dictionary"""
#         gearratio = [0.247343,0.216425,0.193237,0.170048,0.154589,0.131401]
        gearratio = [0.35556,0.31111,0.27778,0.24444,0.22222,0.18889]

        while True:
            try:    
                esp_bike_raw = self.espbike.readline().decode('utf-8').rstrip()            
            except:
                esp_bike_raw = "'data': 'none'"
                
            try:
                esp_hatch_raw = self.esphatch.readline().decode('utf-8').rstrip()
            except:
                esp_hatch_raw = "'data': 'none'"
                
            try:
                esp_gear_raw = self.espgear.readline().decode('utf-8').rstrip()
            except:
                esp_gear_raw = "'data': 'none'"
            
            data_string = ''.join(("{",esp_bike_raw,",",esp_gear_raw,",",esp_hatch_raw,"}"))
            
            try:
                data_dict = ast.literal_eval(data_string)
                #add calculate cadence and total speed in kph
                try:
                    data_dict["cr"] = round(data_dict["s"]*(gearratio[int(data_dict["g"])-1]),2)
                    data_dict["ts"] = round((data_dict["c"]*1.47655)*(60/1000),2)
                except:
                    pass

                break
            except SyntaxError:
                print("data format error: ")
                print(data_string)
                continue
            except ValueError:
                print("value error due to malformed node or string")
                continue
            
        return data_dict
    
    def update_attributes(self, data_as_dict):
        """This method updates the class instance attributes based on the formatted dictionary.
        Additionally, it registers the attribute as 'unavailable' if the sensor fails to send it
        to the pi."""
        available_data = []
        for key in data_as_dict:
            setattr(self, key, data_as_dict[key])
            available_data.append(key)
        unavailable_data = list(set(available_data).symmetric_difference(set(self.expected_data)))
        if len(unavailable_data) > 0:
            unavailable_data_as_dict= dict.fromkeys(unavailable_data, 0)
            for key in unavailable_data_as_dict:
                setattr(self, key, unavailable_data_as_dict[key])
    
    def process(self):
        """This method runs the update_attributes and format_data methods in the appropriate order,
        then returns a list of the current class instance attributes related to sensor data values.
        This data is then sent back to the esp32(bike) for transmission"""
        self.update_attributes(self.format_data())
        current_data = dict(self.__dict__)
        current_data.pop("expected_data")
        current_data.pop("esphatch")
        current_data.pop("espbike")
        current_data.pop("espgear")
        current_data.pop("last_log")
        # testing number of data points
#         current_data.pop("c")
#         current_data.pop("l")
#         current_data.pop("r")
#         current_data.pop("cr")
#         current_data.pop("s")
#         current_data.pop("ts")
#         current_data.pop("g")
#         current_data.pop("sa")
#         current_data.pop("ax")
#         current_data.pop("ay")
#         current_data.pop("az")
#         current_data.pop("vx")
#         current_data.pop("vy")
#         current_data.pop("vz")
#         current_data.pop("t")
#         current_data.pop("p")
#         current_data.pop("h")
#         current_data.pop("b")
        current_data.pop("la")
        current_data.pop("lo")
        current_data.pop("df")
        current_data.pop("active_ports")
        
        print(current_data)

        current_data_line = str(list(current_data.values()))[1:-1]
        current_data_string = f"{current_data_line}\n"
        current_data_bytes = str.encode(current_data_string)
        try:
            self.esphatch.write(current_data_bytes)
            print(f"data sent: {current_data_line}")
        except AttributeError:
            print(f"data packet not sent: {current_data_line}")
        except OSError:
            print(f"Hatch ESP disconnected, transmit error")
            
        return list(current_data.values())

# debugging
# sensor_processor = SensorDataProcessor()
# sensor_processor.check_port_complete(sensor_processor.auto_port())
# stopwatch_start = time.perf_counter_ns()
# sensor_processor.raw_read()
# stopwatch_end = time.perf_counter_ns()
# print("raw read performance time: ")
# print(stopwatch_end - stopwatch_start)
# # while True:
#     sensor_processor.raw_read()




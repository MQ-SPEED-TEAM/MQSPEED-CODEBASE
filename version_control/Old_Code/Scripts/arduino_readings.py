#//////////////////////IMPORTING LIBRARIES/////////////////////////
#//////////////////////////////////////////////////////////////////

import time
import serial


#///////////////////////ARDUNIO SETUP///////////////////////////////
#///////////////////////////////////////////////////////////////////

esphatch = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
esphatch.reset_input_buffer()

espbike = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
espbike.reset_input_buffer()

class HallEffectError(Exception):
    """Raise for hall effectt sensor disconnect"""
    def __init__(self,message):
        super().__init__(message)


class HEF_Error_Finder:
    """Process errors such as hall effect disconnects"""
    def __init__(self):
        self.center_hef = None
        self.right_hef = None
        self.left_hef = None
        self.crank_hef = None
        self.shaft_hef = None
        
    def error_detection(self, center_hef, right_hef, left_hef, crank_hef, shaft_hef):
        self.center_hef = center_hef
        self.right_hef = right_hef
        self.left_hef = left_hef
        self.crank_hef = center_hef
        self.shaft_hef = shaft_hef
        rpm_priority_list = [center_hef, right_hef, left_hef]
        all_sensors = [center_hef, right_hef, left_hef, center_hef, shaft_hef]
        boolean_check = [sensor!=0 for sensor in all_sensors]
        possible_errors = {"center_hef_dead":[False, True, True, True, True],
                           "right_hef_dead":[True, False, True, True, True],
                           "left_hef_dead":[True, True, False, True, True],
                           "crank_hef_dead":[True, True, True, False, True],
                           "shaft_hef_dead":[True, False, True, True, False],
                           "all_live":[True, True, True, True, True]}
        error_key = str(list(possible_errors.keys())[list(possible_errors.values()).index(boolean_check)])
        
        return error_key
    
    def error_response(self):
        pass
    
class HEF_Data_Processor():
    """Manage data sending to main"""
    def __init__(self):
        self.wheel_centre = 0
        self.wheel_right = 0
        self.wheel_left = 0
        self.crank = 0
        self.shaft = 0
        self.gear = 0
        self.Total_Speed = 0
        self.lat = 0
        self.error_response = 0
        
    def set_data(self):
        rpm_line = espbike.readline().decode('utf-8').rstrip()
        Sensors_list = rpm_line.split(",")
        gps_line = esphatch.readline().decode('utf-8').rstrip()
        gps_list = gps_line.split(",")
    
        if(len(Sensors_list)!=6):
           for x in range(6-len(Sensors_list)):
               Sensors_list.append(x)
               
        #/////////////////SENSOR LIST///////////////////////////
        #///////////////////////////////////////////////////////
        self.wheel_centre = Sensors_list[2]
        self.wheel_right = Sensors_list[1]
        self.wheel_left = Sensors_list[0]
        self.crank = Sensors_list[3]
        self.shaft = Sensors_list[4]
        self.gear = 3
        self.Total_Speed = Sensors_list[5]
        self.lat = gps_list[0]

        return [self.wheel_centre, self.wheel_right, self.wheel_left, self.crank,
                self.shaft,self.gear, self.Total_Speed, self.lat]
    
    def send_data(self,wheel_centre, wheel_right, wheel_left, crank,
                  shaft, gear, Total_Speed, lat, error_key):
        self.wheel_centre = wheel_centre
        self.wheel_right = wheel_right
        self.wheel_left = wheel_left
        self.crank = crank
        self.shaft = shaft
        self.gear = gear
        self.Total_Speed = Total_Speed
        self.lat = lat
        self.error_response = error_key
    
    

#/////////////////////MAIN//////////////////////////////
#///////////////////////////////////////////////////////
HEF_error_finder = HEF_Error_Finder()
HEF_data_processor = HEF_Data_Processor()

def main():
    data_list = HEF_data_processor.set_data()
    data_list.append(HEF_error_finder.error_detection(data_list[0],data_list[1],
                                     data_list[2],data_list[3],data_list[4]))
    print(data_list)
    HEF_data_processor.send_data(data_list[0],data_list[1],
                                     data_list[2],data_list[3],data_list[4],data_list[5],
                                 data_list[6],data_list[7],data_list[8])
    
    
    main.wheel_centre = HEF_data_processor.wheel_centre
    main.wheel_right = HEF_data_processor.wheel_right
    main.wheel_left = HEF_data_processor.wheel_left
    main.crank = HEF_data_processor.crank
    main.shaft = HEF_data_processor.shaft
    main.gear = HEF_data_processor.gear
    main.Total_Speed = HEF_data_processor.Total_Speed
    main.lat = HEF_data_processor.lat
    main.error_response = HEF_data_processor.error_response
    
    #print(wheel_centre, wheel_right, wheel_left, crank, shaft, gear, Total_Speed,
          #lat,error_response)
    
# def data():
#     #////////////////////SETUP////////////////////////////////
#     #/////////////////////////////////////////////////////////
#         rpm_line = espbike.readline().decode('utf-8').rstrip()
#         Sensors_list = rpm_line.split(",")
#         
#         gps_line = esphatch.readline().decode('utf-8').rstrip()
#         gps_list = gps_line.split(",")
#     
#         
#         if(len(Sensors_list)!=6):
#            for x in range(6-len(Sensors_list)):
#                Sensors_list.append(x)
#                
#         #/////////////////SENSOR LIST///////////////////////////
#         #///////////////////////////////////////////////////////
#         data.wheel_left = Sensors_list[0]
#         data.wheel_right = Sensors_list[1]
#         data.wheel_centre = Sensors_list[2]
#         data.crank = Sensors_list[3]
#         data.shaft = Sensors_list[4]
#         data.gear = 3
#         data.Total_Speed = Sensors_list[5]
#         data.lat = gps_list[0]
#         
#         
# #     except:
# #         data.wheel_left = 0.00
# #         data.wheel_right = 0.00
# #         data.wheel_centre = 0.00
# #         data.shaft = 0.00
# #         data.crank = 0.00
# #         data.gear = 3
# #         data.Temperature = 0.00
# #         data.Humidity = 0.00
# #         data.Pressure = 0.00
# #         data.Total_Speed = 0.00
# 
#     
# #//def fail_check():
    
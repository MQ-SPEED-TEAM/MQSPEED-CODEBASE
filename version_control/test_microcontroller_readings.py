import time
import ast

example_raw_data_bike_1 = """'center_wheel_rpm':400,'left_wheel_rpm':402,'right_wheel_rpm':399,'crank_rpm':40,'shaft_rpm':401,'gear_set':2,'steering_angle':1"""

example_raw_data_hatch_1 = """'acceleration_x':4,'acceleration_y':1,'acceleration_z':7,'angular_velocity_x':5,'angular_velocity_x':5,'angular_velocity_y':1,'angular_velocity_z':8,'temperature':32,'air_pressure':80,'humidity':5,'latitude':15.00194,'longitude':18.12049,'distance_to_finish':544"""

example_raw_data_bike_2 = """'center_wheel_rpm':400,'left_wheel_rpm':402,'right_wheel_rpm':399,'gear_set':2,'steering_angle':1"""

example_raw_data_hatch_2 = """'acceleration_x':4,'acceleration_y':1,'acceleration_z':7,'angular_velocity_x':5,'angular_velocity_x':5,'angular_velocity_y':1,'angular_velocity_z':8,'temperature':32,'air_pressure':80,'humidity':5"""

class SensorDataProcessor:
    """Takes data from all sensors and packages them for either data logging, screen display, or to send back to microcontrollers"""

    def __init__(self) -> None:
        self.center_wheel_rpm = 0
        self.left_wheel_rpm = 0
        self.right_wheel_rpm = 0
        self.crank_rpm = 0
        self.shaft_rpm = 0
        self.gear_set = 0
        self.steering_angle = 0
        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration_z = 0
        self.angular_velocity_x = 0
        self.angular_velocity_y = 0
        self.angular_velocity_z = 0
        self.temperature = 0
        self.air_pressure = 0
        self.humidity = 0
        self.latitude = 0
        self.longitude = 0
        self.distance_to_finish = 0

        self.expected_data = ["center_wheel_rpm","left_wheel_rpm","right_wheel_rpm","crank_rpm","shaft_rpm","gear_set",
                                 "steering_angle","acceleration_x","acceleration_y","acceleration_z","angular_velocity_x",
                                 "angular_velocity_x","angular_velocity_y","angular_velocity_z","temperature","air_pressure",
                                 "humidity","latitude","longitude","distance_to_finish"]

        self.esphatch = "some string from serial hatch"
        self.espbike = "some string from serial bike"
    

    def format_data(self,data_string_1,data_string_2):
        data_string = ''.join(("{",data_string_1,",",data_string_2,"}"))
        data_dict = ast.literal_eval(data_string)

        return data_dict
    
    def update_attributes(self, data_as_dict):
        available_data = []
        for key in data_as_dict:
            setattr(self, key, data_as_dict[key])
            available_data.append(key)
        unavailable_data = list(set(available_data).symmetric_difference(set(self.expected_data)))
        if len(unavailable_data) > 0:
            unavailable_data_as_dict= dict.fromkeys(unavailable_data, "unavailable")
            for key in unavailable_data_as_dict:
                setattr(self, key, unavailable_data_as_dict[key])
    
    def process(self,data_string_1, data_string_2):
        self.update_attributes(self.format_data(data_string_1, data_string_2))
        current_data = dict(self.__dict__)
        current_data.pop("expected_data")
        current_data.pop("esphatch")
        current_data.pop("espbike")

        return current_data

        
        


Sensor_Data_Processor = SensorDataProcessor()
print(Sensor_Data_Processor.__dict__)

for i in range(10):
    data_input_1 = input(f"Input first espbike data string [test {i+1}]: ")
    data_input_2 = input(f"Input first esphatch data string [test {i+1}]: ")

    print(Sensor_Data_Processor.process(data_input_1,data_input_2))
    


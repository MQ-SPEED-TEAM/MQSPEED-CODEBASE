
//////////////////////////////////////////LIBRARIES/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <RunningAverage.h>
#include <DFRobot_BME680_I2C.h>
#include <Wire.h>
#include "esp_attr.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <AS5600.h>
#define SERIAL_BUFFER_SIZE  2048
String pi_data;
String command;
// 25, 33, 32, 35, 34 (hall effect)
// d2 and d4 (steering angle)
// Connect to the GPS on the hardware port

////libraries stuff
//Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 10

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
////////////////////////////////////////Libraries initialization////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RunningAverage battery_pi_read(1000);
RunningAverage battery_analog_read(1000);

//////////////////////////////////////////////I2C sensors///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DFRobot_BME680_I2C bme(0x77);  //I2C BME680 ID

/////////////////////////////////////////////////VARIABLES///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////SPEED/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long time_l;            // Calculates the time between each sector of the wheel. Time of the magnets in the left wheel
unsigned long time_r;            // Calculates the time between each sector of the wheel. Time of the magnets in the right wheel 
unsigned long time_c;          // Calculates the time between each sector of the wheel. Time of the magnets in the centre wheel 
unsigned long time_cr;           // Calculates the time between each sector of the wheel. Time of the magnets in the crank wheel 
unsigned long time_shaft;            // Calculates the time between each sector of the wheel. Time of the magnets in the centre wheel 
unsigned long time_output;
unsigned long prev_l;            //prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the left wheel 
unsigned long prev_r;           // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the right wheel
unsigned long prev_c;          // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the centre wheel
unsigned long prev_cr;         // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the crank wheel
unsigned long prev_shaft;         // prev_ls the time of the first previous time the magnet was passed through. Time of the magnets in the crank wheel
unsigned long prev_output;
float RPM_L;                     // RPM of the left wheel
float RPM_R;                    // RPM of the right wheel 
float RPM_C;                   // RPM of the centre wheel
float RPM_CRANK;              // RPM of the crank wheel
float RPM_SHAFT;              // RPM of the gear shaft
float average_rpm;            // Average rpm of 3 wheels
float total_speed;           // total speed if the bike 
float some_constant_from_gear_ratio;  // placeholder for gearratio constant
int samples = 0; 
int x = 0; 
int priority_wheel_int = 1;
struct ints_struct{
  int left;
  int right;
  int center;
  int shaft;
  int crank;
  int priority;
};


/////////////////////////////////////////////////STEERING ANGLE VARIABLES///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float steering_angle;
float steering_angle_max;
float steering_angle_min;
float steering_angle_center;

///////////////////////////////////////////////////AIR QUALITY//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float seaLevel;              // SeaLevel value 
float Temperature;          // Temperature value 
float Humidity;            // Humidity value 
float Pressure;           // Pressure value
unsigned long lastRead=0;

////////////////////////////////////////////////////////PINS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int hall_pin = 12;         // Pin relates to wheel left 
const int hall_pin2 = 13;        // Pin relates to wheel right 
const int hall_pin3 = 14;      // Pin relates to wheel centre 
//const int hall_pin4 = 27;      // Pin relates to crank/
const int hall_pin5 = 26;      // Pin relates to gear shaft
const int set_A = 25;
const int set_B = 32;
const int aux = 33;

//////////////////////////////////////////////////// TWEAK variables///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float wheel_circumference = 2.777; //Circumference of wheel 
const int Magnet_Number = 8; // Number of magnets on the tone wheel
const int print_frequency = 10; //milliseconds between serial.print

////////////////////////////////////////////INTERRUPT FUNCTIONS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;


////////////////////////////////////////////////////SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.begin(115200);
  Serial2.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.setRxBufferSize(SERIAL_BUFFER_SIZE);
  pinMode(set_A, OUTPUT);
  pinMode(set_B, OUTPUT);
  pinMode(aux, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(35, INPUT_PULLUP);//GPS fix
  digitalWrite(set_A, LOW);
  digitalWrite(set_B, LOW);

  battery_pi_read.clear();
  battery_analog_read.clear();
  
  

  
  Serial1.begin(115200);

  delay(1000);

///////////////////////////////////////////magnetometer/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Enable auto-gain */
lis2mdl.enableAutoRange(true);
lis2mdl.begin();

///////accelerometer setup//////
accel.begin();
accel.setRange(LSM303_RANGE_4G);
  lsm303_accel_range_t new_range = accel.getRange();
  accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  lsm303_accel_mode_t new_mode = accel.getMode();
  
///////////////////////////////////////////MAIN////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {
   battery_pi_read.addValue(analogRead(26));
   battery_analog_read.addValue(analogRead(27));

   sensors_event_t accel_event; //declare accel event
   sensors_event_t mag_event; //decalre mag event
   accel.getEvent(&accel_event); //capture accelerometer data
   lis2mdl.getEvent(&mag_event);


    //CALCULATE HEADING from magnetometer data
     float heading = (atan2(mag_event.magnetic.y,mag_event.magnetic.x) * 180) / 3.14159;
      if (heading < 0)
      {
      heading = 360 + heading;
    }

    /// CALCULATE VOLTAGES ///
    float voltage_pi  = battery_pi_read.getAverage();
    voltage_pi = map_f(voltage_pi, 0.0, 4095.0, 0.0, 3.3);
    voltage_pi = (voltage_pi*(12.2/2.2)) + 0.535;

    float voltage_analog  = battery_analog_read.getAverage();
    voltage_analog = map_f(voltage_analog, 0.0, 4095.0, 0.0, 3.3);
    voltage_analog = (voltage_analog*(12.2/2.2)) + 0.535;
  
  //////////////////////////////////////////////Print to screen///////////////////////////////////////////////////////////////////////////
  // order in list should always go (left right center crank shaft)
  time_output = millis()-prev_output;
  if (time_output >= print_frequency){
   prev_output = millis();

   Serial.print("h");
   Serial.print(",");
   Serial.print(accel_event.acceleration.x, 2); 
   Serial.print(",");
   Serial.print(accel_event.acceleration.y, 2); 
   Serial.print(",");
   Serial.print(accel_event.acceleration.z, 2); 
   Serial.print(",");
   //print magnetometer
   Serial.print(mag_event.magnetic.x, 2);
   Serial.print(","); 
   Serial.print(mag_event.magnetic.y, 2); 
   Serial.print(",");
   Serial.print(mag_event.magnetic.z, 2);
   Serial.print(",");
   //print environment
   Serial.print(Temperature, 2);
   Serial.print(",");
   Serial.print(Humidity, 2);
   Serial.print(",");
   Serial.print(Pressure, 2);
   Serial.print(",");
   Serial.print(voltage_pi);
   Serial.print(",");
   Serial.print(voltage_analog);
   //print GPS
  //if(!digitalRead(35)){
    Serial.println(Serial1.read());
  //}
  //else Serial.println();





   /// Receieve from pi and send to transceiver
   if(Serial2.available()>0){
      command = Serial2.readStringUntil('\n');
      command.trim();
      if (command.equals("stop")) {
        Serial.print("stop");
        Serial.println();
        }
      }
    if(Serial.available()>0 && !Serial2.available()>0){
      pi_data = Serial.readStringUntil('\n');
      pi_data.trim();
      Serial2.print(pi_data);
      Serial2.println();
      Serial2.flush();
    }
  }
}

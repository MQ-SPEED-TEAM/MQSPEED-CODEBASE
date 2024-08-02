
//////////////////////////////////////////LIBRARIES/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GPS.h>
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
//#define GPSSerial Serial2
#define GPSECHO true
// 25, 33, 32, 35, 34 (hall effect)
// d2 and d4 (steering angle)
// Connect to the GPS on the hardware port
//Adafruit_GPS GPS(&GPSSerial);

////libraries stuff
//Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 10

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

//
/////////////GPS VALUES/////////
//double GetDistance(double lat1, double lon1, double lat2,double lon2) { 
//    double dLat = (lat2 - lat1) * (3.1415926/180);
//    double dLon = (lon2 - lon1)  * (3.1415926/180);
//    double a = (sin(dLat / 2.0) * sin(dLat / 2.0)) + (cos(lat1 *   (3.1415926/180)) * cos(lat2  * (3.1415926/180)) *sin(dLon / 2.0) * sin(dLon / 2.0));
//    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
//    double R = 6371000.0; // Earth's radius in meters
//    double distance = R * c; // Distance in meters
//    return distance;
//}
//double convertToDecimalMin(double curReading){
//  double a = curReading/100; 
//  int modulus = static_cast<int>(a);
//  double b = modulus *100;
//  double DecMinutes = curReading - b; 
//  double c = DecMinutes/60;
//
//  return modulus + c; 
//}

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
const int hall_pin4 = 27;      // Pin relates to crank/
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
  digitalWrite(set_A, LOW);
  digitalWrite(set_B, LOW);

  battery_pi_read.clear();
  battery_analog_read.clear();
  
  

  
//  //Serial2.begin(115200, SERIAL_8N1, 16, 17);
//  GPS.begin(115200);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//  GPS.sendCommand(PGCMD_ANTENNA);
//  delay(1000);
//  GPSSerial.println(PMTK_Q_RELEASE);

//// BATTERY SETUP ////


///////////////////////////////////////////TEMPERATURE SETUP/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
   uint8_t rslt = 1;
   while(!Serial);
   rslt = bme.begin();
   //Serial.println("BME WORKING");
   bme.startConvert();
   bme.update();

///////////////////////////////////////////magnetometer/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Enable auto-gain */
lis2mdl.enableAutoRange(true);
lis2mdl.begin();

///////accelerometer setup//////
accel.begin();
accel.setRange(LSM303_RANGE_4G);
  lsm303_accel_range_t new_range = accel.getRange();
//  switch (new_range) {
//  case LSM303_RANGE_2G:
////    Serial.println("+- 2G");
//    break;
//  case LSM303_RANGE_4G:
////    Serial.println("+- 4G");
//    break;
//  case LSM303_RANGE_8G:
////    Serial.println("+- 8G");
//    break;
//  case LSM303_RANGE_16G:
////    Serial.println("+- 16G");
//    break;
//  }

  accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  lsm303_accel_mode_t new_mode = accel.getMode();
//  switch (new_mode) {
//  case LSM303_MODE_NORMAL:
////    Serial.println("Normal");
//    break;
//  case LSM303_MODE_LOW_POWER:
////    Serial.println("Low Power");
//    break;
//  case LSM303_MODE_HIGH_RESOLUTION:
////    Serial.println("High Resolution");
//    break;
//  }
///////////////////////////////////////////MAIN////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {
//   char c = GPS.read();
//   if (GPSECHO)
////    if (c) Serial.print(c);/
//    if (GPS.newNMEAreceived()) {
////    Serial.print(GPS.lastNMEA()); 
//    if (!GPS.parse(GPS.lastNMEA())) 
//      return; 
//  }
   battery_pi_read.addValue(analogRead(26));
   battery_analog_read.addValue(analogRead(27));

   sensors_event_t accel_event; //declare accel event
   sensors_event_t mag_event; //decalre mag event
   accel.getEvent(&accel_event); //capture accelerometer data
   lis2mdl.getEvent(&mag_event);

     if(millis()-lastRead>1000){
       bme.startConvert();
       bme.update();
       Temperature=bme.readTemperature() / 100, 2;
       Humidity=bme.readHumidity() / 1000, 2;
       Pressure=bme.readPressure();
       lastRead=millis();
     }


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
//   Serial.print("'vx':");
//   Serial.println(mag_event.magnetic.x, 2);
//   float mag_x = round(mag_event.magnetic.x);
//   float test_x = 1.45;
//   binaryFloat hi;
//   hi.floatingPoint = 11.7;
//   Serial.print()
//   Serial.write(test_x);
//   Serial.write((byte*)&test_x,4);/
//   Serial.write(hi.binary,4);


//   int ax_int = accel_event.acceleration.x*100;
//   int ay_int = accel_event.acceleration.y*100;
//   int az_int = accel_event.acceleration.z*100;
//   int mx_int = mag_event.magnetic.x*100;
//   int my_int = mag_event.magnetic.y*100;
//   int mz_int = mag_event.magnetic.z*100;
//   int t_int = Temperature*100;
//   int h_int = Humidity*100;
//   int p_int = Pressure*100;
//   int bp_int = voltage_pi*100;
//   int ba_int = voltage_analog*100;
//
//   Serial.write(ax_int);
//   Serial.write(ay_int);
//   Serial.write(az_int);
//   Serial.write(mx_int);
//   Serial.write(mx_int);
//   Serial.write(mx_int);
//   Serial.write(t_int);
//   Serial.write(h_int);
//   Serial.write(p_int);
//   Serial.write(bp_int);
//   Serial.write(ba_int);
//   Serial.print("\n");
//   Serial.flush();
   




   
//   Serial.print("s");
//   Serial.flush();
//  float ax = accel_event.acceleration.x;
//  float ay = accel_event.acceleration.z;
//  float az = accel_event.acceleration.y;
//  float vx = mag_event.magnetic.x;
//  float vy = mag_event.magnetic.y;
//  float vz = mag_event.magnetic.z;
//  float t = Temperature;
//  float h = Humidity;
//  float p = Pressure;
//  float vp = voltage_pi;
//  float vb = voltage_analog;

///
   Serial.print("h");
   Serial.print(",");
   Serial.print(accel_event.acceleration.x, 2); 
   Serial.print(",");
   Serial.print(accel_event.acceleration.z, 2); 
   Serial.print(",");
   Serial.flush();
   Serial.print(accel_event.acceleration.y, 2); 
   Serial.print(",");
   //print magnetometer
   Serial.print(mag_event.magnetic.x, 2);
   Serial.print(","); 
   Serial.print(mag_event.magnetic.y, 2); 
   Serial.print(",");
   Serial.flush();
   Serial.print(mag_event.magnetic.z, 2);
   Serial.print(",");
   Serial.flush();
   //print environment
   Serial.print(Temperature, 2);
   Serial.print(",");
   Serial.print(Humidity, 2);
   Serial.print(",");
   Serial.flush();
   Serial.print(Pressure, 2);
   Serial.print(",");
   Serial.print(voltage_pi);
   Serial.print(",");
   Serial.print(voltage_analog);
   Serial.print("\n");// this gives the reading in voltages 
   Serial.flush();


   /// Receieve from pi and send to transceiver
   if(Serial2.available()>0){
      command = Serial2.readStringUntil('\n');
      command.trim();
      if (command.equals("stop")) {
        Serial.print("stop");
        Serial.println();
        Serial.flush();
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

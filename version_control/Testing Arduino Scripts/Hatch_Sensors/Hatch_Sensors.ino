
//////////////////////////////////////////LIBRARIES/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GPS.h>
#include <RunningAverage.h>
#include <DFRobot_BME680_I2C.h>
#include <Wire.h>
#include <esp_attr.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>

#define SERIAL_BUFFER_SIZE  2048

String pi_data;
String command;
#define GPSECHO true

////libraries stuff
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 10

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

Adafruit_GPS GPS(&Serial1);
#define GPSECHO false

////////////////////////////////////////Libraries initialization////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RunningAverage battery_pi_read(1000);
RunningAverage battery_analog_read(1000);

//////////////////////////////////////////////I2C sensors///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DFRobot_BME680_I2C bme(0x77);  //I2C BME680 ID

/////////////////////////////////////////////////VARIABLES///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long time_output;
unsigned long prev_output;
unsigned long led_time;
unsigned long led_prev_time = 0;
int led_state = LOW;
int x = 0; 
float heading = 0;
float voltage_pi = 0;
float voltage_analog = 0;

///////////////////////////////////////////////////AIR QUALITY//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Temperature;          // Temperature value 
float Humidity;            // Humidity value 
float Pressure;           // Pressure value
unsigned long lastRead=0;

////////////////////////////////////////////////////////PINS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int set_A = 25;           //transciever aux
const int set_B = 32;           //transciever aux
const int aux = 33;             //transciever aux
#define LED 2                   //ESP led pin
#define pi_bat 27                  //Pi battery
#define backup_bat 26                  //Backup screen battery

//////////////////////////////////////////////////// TWEAK variables///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int print_frequency = 10; //milliseconds between Prints

////////////////////////////////////////////////////SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200); //Setup USB communication port
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  
  Serial2.begin(115200);  //Setup Transciever communication port
  Serial2.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial2.setRxBufferSize(SERIAL_BUFFER_SIZE);

  Serial1.begin(9600, SERIAL_8N1, 18, 19); //Setup GPS communication port

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  
  pinMode(set_A, OUTPUT);
  pinMode(set_B, OUTPUT);
  pinMode(LED,OUTPUT);
  pinMode(aux, INPUT);
  pinMode(backup_bat, INPUT);
  pinMode(pi_bat, INPUT);
  
  digitalWrite(set_A, LOW);
  digitalWrite(set_B, LOW);
  
  battery_pi_read.clear();
  battery_analog_read.clear();

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

  accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
  lsm303_accel_mode_t new_mode = accel.getMode();

///////////////////////////////////////////MAIN////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


float map_f(float x, float in_min, float in_max, float out_min, float out_max) {/////////WTF is this?
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {
   battery_pi_read.addValue(analogRead(pi_bat));
   battery_analog_read.addValue(analogRead(backup_bat));

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
     heading = (atan2(mag_event.magnetic.y,mag_event.magnetic.x) * 180) / 3.14159;
      if (heading < 0)
      {
      heading = 360 + heading;
    }

    /// CALCULATE VOLTAGES ///
    voltage_pi  = battery_pi_read.getAverage();
    voltage_pi = map_f(voltage_pi, 0.0, 4095.0, 0.0, 3.3);
    voltage_pi = (voltage_pi*(12.2/2.2)) + 0.535;

    voltage_analog  = battery_analog_read.getAverage();
    voltage_analog = map_f(voltage_analog, 0.0, 4095.0, 0.0, 3.3);
    voltage_analog = (voltage_analog*(12.2/2.2)) + 0.535;

  if(voltage_pi <= 14.1){
    led_time = millis();
    if (led_time - led_prev_time >= 1000) {
      // if the LED is off turn it on and vice-versa:
      led_state = (led_state == LOW) ? HIGH : LOW;
  
      // set the LED with the ledState of the variable:
      digitalWrite(LED, led_state);
  
      // save the last time you blinked the LED
      led_prev_time = led_time;
    }
  } 
  else {
    digitalWrite(LED, LOW);
  }

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  //////////////////////////////////////////////Print to screen///////////////////////////////////////////////////////////////////////////
  time_output = millis()-prev_output;
   if (time_output >= print_frequency){
   prev_output = millis();
   Serial.print("h");
   Serial.print(",");
   Serial.print(accel_event.acceleration.x, 2); 
   Serial.print(",");
   Serial.print(accel_event.acceleration.z, 2); 
   Serial.print(",");
   Serial.print(accel_event.acceleration.y, 2); 
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
   Serial.print(voltage_pi);// this gives the reading in voltages 
   Serial.print(",");
   Serial.print(voltage_analog);// this gives the reading in voltages 
   Serial.print(",");

   //GPS write
   if(GPS.fix){
      Serial.print(GPS.latitude, 4); 
      Serial.print(GPS.lat);
      Serial.print(",");
      Serial.print(GPS.longitude, 4); 
      Serial.print(GPS.lon);
      Serial.print(",");
      Serial.print(GPS.speed*1.852);
      Serial.print(",");
      Serial.print(GPS.altitude);
      Serial.print(",");
      Serial.print((int)GPS.satellites);
   }
   else{Serial.print("0,0,0,0,0");}
   
   Serial.print("\n");
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

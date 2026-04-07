
//////////////////////////////////////////LIBRARIES/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GPS.h>
#include <RunningAverage.h>
#include <DFRobot_BME680_I2C.h>
#include <Wire.h>
#include <esp_attr.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h> // Library name is BNO08x but actual sensor is FSM30x


#define SERIAL_BUFFER_SIZE  2048

// ===== I2C PINS =====
#define SDA_PIN 21
#define SCL_PIN 22

// Optional pins (can be -1 if unused)
#define BNO08X_INT -1
#define BNO08X_RST -1


String pi_data;
String command;
#define GPSECHO true

////libraries stuff
Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;

Adafruit_GPS GPS(&Serial1);
#define GPSECHO false

////////////////////////////////////////Libraries initialization////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RunningAverage battery_pi_read(1000);
RunningAverage battery_analog_read(1000);
RunningAverage averaged_roll(100);
RunningAverage averaged_pitch(100);
RunningAverage averaged_yaw(100);

//////////////////////////////////////////////I2C sensors///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DFRobot_BME680_I2C bme(0x77);  //I2C BME680 ID/

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
float instant_roll;
float instant_pitch;
float instant_yaw;
float qw;
float qx;
float qy;
float qz;

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
  delay(700);
  Serial.setTxBufferSize(SERIAL_BUFFER_SIZE);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
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
  averaged_roll.clear();
  averaged_pitch.clear();
  averaged_yaw.clear();

  
///////////////////////////////////////////TEMPERATURE SETUP/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


   uint8_t rslt = 1;
   while(!Serial);
   delay(400);
   rslt = bme.begin();
   //Serial.println("BME WORKING");
   bme.startConvert();
   bme.update();

   delay(400);
//////////IMU SETUP////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // BNO08X supports 400kHz
  delay(500);

  if (!bno08x.begin_I2C()) {
    Serial.println("❌ BNO08X not detected!");
    while (1) delay(10);
  }
  delay(300);
  
  // Enable rotation vector
 if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) { // 10 ms
    Serial.println("Failed to enable rotation vector");
}



///////////////////////////////////////////MAIN////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


float map_f(float x, float in_min, float in_max, float out_min, float out_max) {/////////WTF is this?
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {
   battery_pi_read.addValue(analogRead(pi_bat));
   battery_analog_read.addValue(analogRead(backup_bat));

    // Calculate roll pitch yaw
    
    if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      qw = sensorValue.un.rotationVector.real;
      qx = sensorValue.un.rotationVector.i;
      qy = sensorValue.un.rotationVector.j;
      qz = sensorValue.un.rotationVector.k;

      // Quaternion from IMU
      float qw_i = qw;
      float qx_i = qx;
      float qy_i = qy;
      float qz_i = qz;

      // Rotation quaternion for mounting orientation
      const float s = -0.70710678; // -sqrt(2)/2
      float qw_r = 0.0f;
      float qx_r = s;
      float qy_r = s;
      float qz_r = 0.0f;

      // Apply mounting correction
      float qw_m = qw_r*qw_i - qx_r*qx_i - qy_r*qy_i - qz_r*qz_i;
      float qx_m = qw_r*qx_i + qx_r*qw_i + qy_r*qz_i - qz_r*qy_i;
      float qy_m = qw_r*qy_i - qx_r*qz_i + qy_r*qw_i + qz_r*qx_i;
      float qz_m = qw_r*qz_i + qx_r*qy_i - qy_r*qx_i + qz_r*qw_i;

      // Replace IMU quaternion with corrected one
      qw = qw_m;
      qx = qx_m;
      qy = qy_m;
      qz = qz_m;

      // Convert to Euler angles (degrees)
      instant_roll  = atan2(2.0 * (qw * qx + qy * qz),
                          1.0 - 2.0 * (qx * qx + qy * qy));
      instant_pitch = asin(2.0 * (qw * qy - qz * qx));
      instant_yaw   = atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
      
      instant_roll  *= 180.0 / PI; //Rotation about X (forward+ and backwards- axis)
      instant_pitch *= 180.0 / PI; //Rotation about Y (left+ and right- axis)
      instant_yaw   *= 180.0 / PI; //Rotation about Z (up+ and down- axis)

      averaged_roll.addValue(instant_roll);
      averaged_pitch.addValue(instant_pitch);
      averaged_yaw.addValue(instant_yaw);

      }
    }
 
    // Calculate BME data
     if(millis()-lastRead>1000){
       bme.startConvert();
       bme.update();
       Temperature=bme.readTemperature() / 100, 2;
       Humidity=bme.readHumidity() / 1000, 2;
       Pressure=bme.readPressure();
       lastRead=millis();
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

   // Parse GPS data
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
   Serial.print(roll, 2); 
   Serial.print(",");
   Serial.print(pitch, 2); 
   Serial.print(",");
   Serial.print(yaw, 2); 
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
   else{Serial.print("0n,0e,0,0,0");}
   
   Serial.print("0\n");
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
